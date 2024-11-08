#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>

geometry_msgs::Quaternion target_quaternion;
geometry_msgs::Quaternion current_quaternion;
float current_x, current_y, aim_x, aim_y,target_distance,current_distance;
bool rotate= false;
bool stop= false;
ros::Publisher pub; // 定义为全局变量
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void init();

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_node");
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber sub = n.subscribe("odom", 10, odomCallback);
    ros::Subscriber sub2=n.subscribe("scan", 10, scanCallback);
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
    target_distance = 0.3;
    current_distance =0.5;
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
    else if (!stop && rotate) {
        twist.linear.x = 0.15;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
    }
    if (stop) {
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
    pub.publish(twist);
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        current_distance = msg->ranges[0];
        geometry_msgs::Twist twist;

        if (current_distance <= target_distance) {
            // 当小车距离前方为0.3米时，将速度归0，停下小车
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;
            stop = true; // 设置停止标志
        }

   else  if (!stop) {
        twist.linear.x = 0.15;
        twist.angular.z = 0;
    }
        pub.publish(twist);
    }
