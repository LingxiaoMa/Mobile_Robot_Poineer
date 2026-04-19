/*
 *  ROS2 node for Pioneer robot via AriaCoda.
 *  Subscribes: /cmd_vel
 *  Publishes:  /odom  +  TF odom→base_link  (via 20 Hz timer)
 *
 *  Run: ros2 run ariaNode ariaNode -rp /dev/ttyUSB0
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "Aria/Aria.h"

bool stopRunning = false;

using namespace std::chrono_literals;

class ariaNode : public rclcpp::Node {
public:
    ariaNode(float* forwardSpeed, float* rotationSpeed, ArRobot* robot)
        : Node("Aria_node"), _tf_broadcaster(this)
    {
        _forwardSpeed  = forwardSpeed;
        _rotationSpeed = rotationSpeed;
        _robot         = robot;

        _cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&ariaNode::_cmdVelCb, this, std::placeholders::_1));

        _odomPub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Publish odom via timer so it runs inside spin_some, not in the tight loop
        _odomTimer = create_wall_timer(50ms,
            std::bind(&ariaNode::_publishOdom, this));
    }

private:
    void _cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
        *_forwardSpeed  = static_cast<float>(msg->linear.x);
        *_rotationSpeed = static_cast<float>(msg->angular.z);
    }

    void _publishOdom() {
        _robot->lock();
        double x_m      = _robot->getX()      / 1000.0;
        double y_m      = _robot->getY()      / 1000.0;
        double th_rad   = _robot->getTh()     * M_PI / 180.0;
        double vx_ms    = _robot->getVel()    / 1000.0;
        double vth_rads = _robot->getRotVel() * M_PI / 180.0;
        _robot->unlock();

        auto now = this->get_clock()->now();

        // TF: odom → base_link
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = now;
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "base_link";
        tf.transform.translation.x = x_m;
        tf.transform.translation.y = y_m;
        tf.transform.rotation.z = std::sin(th_rad / 2.0);
        tf.transform.rotation.w = std::cos(th_rad / 2.0);
        _tf_broadcaster.sendTransform(tf);

        // Odometry message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";
        odom.pose.pose.position.x = x_m;
        odom.pose.pose.position.y = y_m;
        odom.pose.pose.orientation.z = tf.transform.rotation.z;
        odom.pose.pose.orientation.w = tf.transform.rotation.w;
        odom.twist.twist.linear.x  = vx_ms;
        odom.twist.twist.angular.z = vth_rads;
        _odomPub->publish(odom);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmdVelSub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       _odomPub;
    rclcpp::TimerBase::SharedPtr                                _odomTimer;
    tf2_ros::TransformBroadcaster                               _tf_broadcaster;
    float*    _forwardSpeed;
    float*    _rotationSpeed;
    ArRobot*  _robot;
};

void my_handler(int s) {
    printf("Caught signal %d\n", s);
    stopRunning = true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    Aria::init();
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArRobot* robot = new ArRobot();

    signal(SIGINT, my_handler);

    ArRobotConnector robotConnector(&parser, robot);
    if (!robotConnector.connectRobot()) {
        ArLog::log(ArLog::Terse, "Could not connect to robot.");
        if (parser.checkHelpAndWarnUnparsed()) {
            Aria::logOptions();
            Aria::exit(1);
        }
    }

    robot->setAbsoluteMaxTransVel(400);
    robot->runAsync(true);
    robot->enableMotors();

    float forwardSpeed  = 0.0f;
    float rotationSpeed = 0.0f;

    auto node = std::make_shared<ariaNode>(&forwardSpeed, &rotationSpeed, robot);

    while (!stopRunning) {
        rclcpp::spin_some(node);   // fires odom timer + processes cmd_vel

        robot->lock();
        robot->setVel(forwardSpeed * 500.0);
        robot->setRotVel(rotationSpeed * 50.0);
        robot->unlock();
    }

    robot->disableMotors();
    robot->stopRunning();
    robot->waitForRunExit();
    Aria::exit(0);
    return 0;
}
