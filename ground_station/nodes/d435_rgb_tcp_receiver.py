#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class D435RgbTcpReceiver:
    """
    DK2500 端节点：
    1. 开 TCP Server
    2. 接收 Jetson 发来的 JPEG 图像
    3. 解码成 OpenCV 图像
    4. 发布为 ROS Image
    """

    def __init__(self):
        rospy.init_node("d435_rgb_tcp_receiver", anonymous=True)

        self.listen_ip = rospy.get_param("~listen_ip", "0.0.0.0")
        self.listen_port = rospy.get_param("~listen_port", 9200)
        self.pub_topic = rospy.get_param(
            "~pub_topic",
            "/d435/rgb_from_jetson"
        )

        self.show_image = rospy.get_param("~show_image", False)

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.pub_topic, Image, queue_size=1)

        rospy.loginfo("D435 RGB TCP Receiver started.")
        rospy.loginfo("Listening on %s:%d", self.listen_ip, self.listen_port)
        rospy.loginfo("Publish topic: %s", self.pub_topic)

    def recv_exact(self, conn, size):
        data = b""
        while len(data) < size:
            chunk = conn.recv(size - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def run(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.listen_ip, self.listen_port))
        server.listen(1)

        header_size = struct.calcsize("!I d H H I")

        while not rospy.is_shutdown():
            rospy.loginfo("Waiting for Jetson connection...")
            conn, addr = server.accept()
            rospy.loginfo("Jetson connected: %s", str(addr))

            try:
                while not rospy.is_shutdown():
                    magic = self.recv_exact(conn, 4)
                    if magic is None:
                        break

                    if magic != b"RGBD":
                        rospy.logwarn("Invalid magic header.")
                        break

                    header = self.recv_exact(conn, header_size)
                    if header is None:
                        break

                    seq, stamp, width, height, payload_len = struct.unpack(
                        "!I d H H I",
                        header
                    )

                    jpg_bytes = self.recv_exact(conn, payload_len)
                    if jpg_bytes is None:
                        break

                    jpg_array = np.frombuffer(jpg_bytes, dtype=np.uint8)
                    frame = cv2.imdecode(jpg_array, cv2.IMREAD_COLOR)

                    if frame is None:
                        rospy.logwarn("JPEG decode failed.")
                        continue

                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    msg.header.stamp = rospy.Time.from_sec(stamp)
                    msg.header.frame_id = "d435_color_from_jetson"

                    self.pub.publish(msg)

                    rospy.loginfo_throttle(
                        2.0,
                        "RGB received: seq=%d, size=%.2f KB, resolution=%dx%d",
                        seq,
                        payload_len / 1024.0,
                        width,
                        height
                    )

                    if self.show_image:
                        cv2.imshow("D435 RGB From Jetson", frame)
                        cv2.waitKey(1)

            except Exception as e:
                rospy.logwarn("Connection error: %s", str(e))

            finally:
                conn.close()
                rospy.logwarn("Jetson disconnected. Waiting for reconnect...")


if __name__ == "__main__":
    receiver = D435RgbTcpReceiver()
    receiver.run()
