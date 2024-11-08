#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

geometry_msgs::Quaternion target_quaternion;
geometry_msgs::Quaternion current_quaternion;
float current_x, current_y, aim_x, aim_y;
bool rotate= false;
ros::Publisher pub; // 定义为全局变量
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void init();

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_node");
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber sub = n.subscribe("odom", 10, odomCallback);
    init();
    ros::Rate r(30);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

void init() {
    target_quaternion = tf::createQuaternionMsgFromYaw(-M_PI / 2);
    current_quaternion = tf::createQuaternionMsgFromYaw(0);
    current_x = 0;
    current_y = -0.49;
    aim_x = 0.55;
    aim_y = -1.64;
}

// 回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 更新当前位置和方向
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    current_quaternion = msg->pose.pose.orientation;
    double roll, pitch, yaw;
    geometry_msgs::Twist twist;
    tf::Quaternion q(current_quaternion.x, current_quaternion.y, current_quaternion.z, current_quaternion.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 移动到A点
    if (!rotate && current_x < aim_x) {
        twist.linear.x = 0.15;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
    }
    // 旋转到特定角度
    else if (!rotate && current_x >= aim_x) {
        if (fabs(yaw + M_PI / 2) < 0.1) { // 检查是否旋转了90度
            rotate = true;
                    } else {
            twist.linear.x = 0;
            twist.angular.z = -0.1; // 顺时针旋转
        }
    }
    // 移动到B点
    else if (rotate && current_y >aim_y) {
        twist.linear.x = 0.15;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
    }
    else {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
    }
    pub.publish(twist);
}