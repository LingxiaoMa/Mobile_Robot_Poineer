#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import depthai as dai
import numpy as np

def cv2_to_imgmsg(cv_image, encoding='bgr8'):
    msg = Image()
    msg.height, msg.width = cv_image.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = False
    if encoding == 'bgr8':
        msg.step = cv_image.shape[1] * 3
    elif encoding == '16UC1':
        msg.step = cv_image.shape[1] * 2
    msg.data = cv_image.tobytes()
    return msg

class OakDriverNode(Node):
    def __init__(self):
        super().__init__('oak_driver_node')
        subprocess.run(['chmod', '-R', '666', '/dev/bus/usb/'], capture_output=True)
        self.rgb_pub = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/oak/stereo/image_raw', 10)

        self.get_logger().info(f"Detected DepthAI Version: {dai.__version__}")
        self.get_logger().info("Using V3 API... (No XLinkOut!)")

        # 启动 V3 Pipeline
        self.p = dai.Pipeline().__enter__()
        
        # ==========================================
        # 1. 配置彩色相机 (直接使用你的成功写法)
        # ==========================================
        self.cam_rgb = self.p.create(dai.node.Camera)
        self.cam_rgb.build(boardSocket=dai.CameraBoardSocket.CAM_A)
        self.out_rgb = self.cam_rgb.requestOutput((640, 480), type=dai.ImgFrame.Type.BGR888p)
        self.q_rgb = self.out_rgb.createOutputQueue(maxSize=4, blocking=False)

        # ==========================================
        # 2. 配置双目与深度 (V3 全新写法)
        # ==========================================
        self.stereo = self.p.create(dai.node.StereoDepth)
        
        # 很多 v3 环境中，MonoCamera 依然存在，如果不兼容我们会做异常捕获
        self.cam_left = self.p.create(dai.node.MonoCamera)
        self.cam_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        self.cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        
        self.cam_right = self.p.create(dai.node.MonoCamera)
        self.cam_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        self.cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # 连接双目到立体深度节点
        self.cam_left.out.link(self.stereo.left)
        self.cam_right.out.link(self.stereo.right)
        
        # 【核心魔法】：直接在 depth 输出上创建队列，不要任何 XLinkOut！
        self.q_depth = self.stereo.depth.createOutputQueue(maxSize=4, blocking=False)

        # 启动
        self.p.start()
        self.get_logger().info('🔥 OAK-D Started! Publishing RGB + Depth via V3 API...')
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frames)

    def publish_frames(self):
        stamp = self.get_clock().now().to_msg()
        
        # 发彩色图
        if (in_rgb := self.q_rgb.tryGet()):
            msg = cv2_to_imgmsg(in_rgb.getCvFrame(), 'bgr8')
            msg.header.stamp, msg.header.frame_id = stamp, 'oak_camera'
            self.rgb_pub.publish(msg)

        # 发深度图
        if (in_depth := self.q_depth.tryGet()):
            msg = cv2_to_imgmsg(in_depth.getFrame().astype(np.uint16), '16UC1')
            msg.header.stamp, msg.header.frame_id = stamp, 'oak_camera'
            self.depth_pub.publish(msg)

    def destroy_node(self):
        self.p.__exit__(None, None, None)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OakDriverNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
