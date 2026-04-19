#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Imu
import math

def main(args=None):
    # 第一步：先让 ROS 2 完全启动并创建 Node，独占底层资源！
    rclpy.init(args=args)
    node = rclpy.create_node('phidget_imu_node')
    pub = node.create_publisher(Imu, '/imu/data_raw', 10)

    # 第二步：等 ROS 2 稳定后，再导入 Phidgets 库，完美避开冲突！
    from Phidget22.Devices.Spatial import Spatial
    from Phidget22.PhidgetException import PhidgetException

    spatial = Spatial()

    # 第三步：定义收到 IMU 数据时的回调逻辑
    def on_spatial_data(self_spatial, acceleration, angularRate, magneticField, timestamp):
        msg = Imu()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # 加速度转换: g -> m/s^2
        g = 9.80665
        msg.linear_acceleration.x = acceleration[0] * g
        msg.linear_acceleration.y = acceleration[1] * g
        msg.linear_acceleration.z = acceleration[2] * g
        msg.linear_acceleration_covariance[0] = -1.0

        # 角速度转换: deg/s -> rad/s
        deg2rad = math.pi / 180.0
        msg.angular_velocity.x = angularRate[0] * deg2rad
        msg.angular_velocity.y = angularRate[1] * deg2rad
        msg.angular_velocity.z = angularRate[2] * deg2rad
        msg.angular_velocity_covariance[0] = -1.0
        
        msg.orientation_covariance[0] = -1.0

        pub.publish(msg)

    # 第四步：挂载回调并尝试连接
    spatial.setOnSpatialDataHandler(on_spatial_data)
    node.get_logger().info('正在尝试连接 Phidgets IMU...')
    
    try:
        spatial.openWaitForAttachment(5000)
        node.get_logger().info('✅ 成功连接到 Phidgets IMU！开始疯狂发布 /imu/data_raw ...')
        spatial.setDataInterval(10)
    except PhidgetException as e:
        node.get_logger().error(f'连接失败，请检查 USB 是否插好: {e.details}')
        return

    # 第五步：进入循环，保持运行
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 安全退出
        try:
            spatial.close()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
