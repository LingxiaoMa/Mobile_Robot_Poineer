#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage
import sys
import numpy as np

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        # 订阅彩色相机话题
        self.sub = self.create_subscription(Image, '/oak/rgb/image_raw', self.callback, 10)
        self.get_logger().info('等待接收 /oak/rgb/image_raw 话题上的图像...')

    def callback(self, msg):
        self.get_logger().info('收到图像数据！正在保存...')
        
        # 1. 还原一维数组到三维 (高度, 宽度, 通道数)
        img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        
        # 2. OAK-D 发出来的是 BGR 格式，存成图片需要转成 RGB
        # numpy 切片反转通道顺序：img_array[:, :, ::-1]
        rgb_array = img_array[:, :, ::-1]
        
        # 3. 使用 Python 原生自带的 PIL 库保存，完美绕过 OpenCV 报错！
        pil_img = PILImage.fromarray(rgb_array)
        save_path = '/workspace/topic_test_photo.jpg'
        pil_img.save(save_path)
        
        self.get_logger().info(f'🎉 成功保存照片至：{save_path}')
        
        # 拍完一张就自动退出
        sys.exit(0)

def main():
    rclpy.init()
    node = ImageSaver()
    try: rclpy.spin(node)
    except SystemExit: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()
