import time
import cv2
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # 参数
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('fps', 60)

        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value

        # 初始化相机
        # self.video_device = f'/dev/video{self.camera_id}'
        # self.cap = cv2.VideoCapture(self.video_device)
        self.cap = cv2.VideoCapture(self.camera_id)

        # 设置相机参数
        self.set_camera_parameters()

        # 创建发布者
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)

        self.bridge = CvBridge()

        self.loop() # 启动循环

    def set_camera_parameters(self):
        # 设置相机的宽度和高度
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))  # 设置编码格式
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # 检查设置是否成功
        print(f"设置的相机: {self.camera_id} 号相机")
        print(f"设置的分辨率: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)} x {self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
        print(f"设置的帧率: {self.cap.get(cv2.CAP_PROP_FPS)}")

    def loop(self):

        while not self.cap.isOpened():
            continue

        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                # 显示图像
                if os.environ.get('DISPLAY') and os.isatty(0):
                    cv2.namedWindow("raw", cv2.WINDOW_NORMAL)
                    cv2.imshow("raw", frame)

                # 转换为 ROS 图像消息
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.image_pub.publish(ros_image)
            
            time.sleep(0.001)  # 避免 CPU 占用过多
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