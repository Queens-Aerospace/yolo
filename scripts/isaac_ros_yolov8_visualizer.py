#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# This script listens for images and object detections on the image,
# then renders the output boxes on top of the image and publishes
# the result as an image message

import cv2
import cv_bridge
import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge


names = {
        0: 'helipad'
}


class Yolov8Visualizer(Node):
    QUEUE_SIZE = 10
    color = (0, 255, 0)
    bbox_thickness = 2

    

    def __init__(self):
        super().__init__('yolov8_visualizer')
        self._bridge = cv_bridge.CvBridge()
        self._processed_image_pub = self.create_publisher(
            Image, 'yolov8_processed_image',  self.QUEUE_SIZE)

        self._detections_subscription = message_filters.Subscriber(
            self,
            Detection2DArray,
            'detections_output') #Dont change baby girl
        self._image_subscription = message_filters.Subscriber(
            self,
            Image,
            'image')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.listener_callback_camera,
            10)
        self.publisher = self.create_publisher(Image, '/image', 10)
        self.bridge = CvBridge()
        
        self._xyz_pub = self.create_publisher(
            Point, 'xyz_coordinates', self.QUEUE_SIZE) 
        

        self.time_synchronizer = message_filters.TimeSynchronizer(
            [self._detections_subscription, self._image_subscription],
            self.QUEUE_SIZE)


        self.time_synchronizer.registerCallback(self.detections_callback)

        self.subscription = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/left/camera_info',
            self.listener_callback,
            10)
        self.subscription 

        self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            10)


    def listener_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        # self.get_logger().info(f'Camera fx: {msg.k[0]}, fy: {msg.k[4]}, cx: {msg.k[2]}, cy: {msg.k[5]}')


    def depth_callback(self, msg):
        # Convert ROS depth image to OpenCV format
        self.cv_depth = self._bridge.imgmsg_to_cv2(msg, "32FC1")

    def listener_callback_camera(self, data):
        # Convert the incoming image from ROS message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgra8')
        
        # Convert BGRA to BGR (this drops the alpha channel)
        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
        
        # Resize the image
        resized_image = cv2.resize(bgr_image, (608, 608))
        
        # Convert the processed image back to a ROS image message
        image_message = self.bridge.cv2_to_imgmsg(resized_image, encoding="bgr8")
        
        # Publish the image
        self.publisher.publish(image_message)
     

    def detections_callback(self, detections_msg, img_msg):
        txt_color = (255, 0, 255)
        cv2_img = self._bridge.imgmsg_to_cv2(img_msg)
        for detection in detections_msg.detections:
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            width = detection.bbox.size_x
            height = detection.bbox.size_y

            label = names[int(detection.results[0].hypothesis.class_id)]
            conf_score = detection.results[0].hypothesis.score
            label = f'{label} {conf_score:.2f}'

            min_pt = (round(center_x - (width / 2.0)),
                      round(center_y - (height / 2.0)))
            max_pt = (round(center_x + (width / 2.0)),
                      round(center_y + (height / 2.0)))
            
            middle_x = (min_pt[0] + max_pt[0]) / 2
            middle_y = (min_pt[1] + max_pt[1]) / 2

            pix_unscaled = [middle_x, middle_y]

            # cv_depth = self._bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            

            w_scale = 3.1578
            h_scale = 1.7763

            U = int(pix_unscaled[0]*w_scale)

            V = int(pix_unscaled[1]*h_scale)


            pix = [U, V]

            depth_value = self.cv_depth[int(middle_x), int(middle_y)]

            self.get_logger().info(f'Depth at ({U},{V}): {depth_value} meters')
            # self.get_logger().info(f"pix_unscaled: {pix_unscaled}")
            # self.get_logger().info(f"pix: {pix}")

            # depth_map = sl.Mat()

            # Z = depth_map.get_value(U, V)

            X = (U - self.cx) * depth_value / self.fx
            Y = (V - self.cy) * depth_value / self.fy

            xyz_msg = Point()
            xyz_msg.x = X
            xyz_msg.y = Y
            xyz_msg.z = float(depth_value)
            self._xyz_pub.publish(xyz_msg) 


            self.get_logger().info(f"X: {X}, Y: {Y}")

            
            lw = max(round((img_msg.height + img_msg.width) / 2 * 0.003), 2)  # line width
            tf = max(lw - 1, 1)  # font thickness
            # text width, height
            w, h = cv2.getTextSize(label, 0, fontScale=lw / 3, thickness=tf)[0]
            outside = min_pt[1] - h >= 3

            cv2.rectangle(cv2_img, min_pt, max_pt,
                          self.color, self.bbox_thickness)
            cv2.putText(cv2_img, label, (min_pt[0], min_pt[1]-2 if outside else min_pt[1]+h+2),
                        0, lw / 3, txt_color, thickness=tf, lineType=cv2.LINE_AA)

        processed_img = self._bridge.cv2_to_imgmsg(
            cv2_img, encoding=img_msg.encoding)
        self._processed_image_pub.publish(processed_img)


def main():
    rclpy.init()
    rclpy.spin(Yolov8Visualizer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
