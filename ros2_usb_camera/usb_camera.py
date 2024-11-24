import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge

import v4l2
import fcntl
import os

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # 参数
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 30)

        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value

        # 初始化相机
        self.video_device = f'/dev/video{self.camera_id}'
        self.cap = cv2.VideoCapture(self.video_device)

        # 设置相机参数
        self.set_camera_parameters()

        # 创建发布者
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        self.bridge = CvBridge()

    def set_camera_parameters(self):
        # 设置相机的宽度和高度
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 转换为 ROS 图像消息
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(ros_image)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()