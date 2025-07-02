#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>

class FilterNode
{
public:
    FilterNode(ros::NodeHandle &nh)
    {
        odom_sub_.subscribe(nh, "/odom", 10);
        imu_sub_.subscribe(nh, "/imu", 10);
        cmd_vel_sub_ = nh.subscribe("/cmd_vel", 10, &FilterNode::cmdVelCallback, this);

        sync_.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>(odom_sub_, imu_sub_, 10));
        sync_->registerCallback(boost::bind(&FilterNode::sensorCallback, this, _1, _2));

        pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/prediction_KF", 10);
    }

private:
    void sensorCallback(const nav_msgs::Odometry::ConstPtr &odom_msg, const sensor_msgs::Imu::ConstPtr &imu_msg)
    {   
        ros::Time current_time = odom_msg->header.stamp;
        double dt = (last_time_.isZero()) ? 0.1 : (current_time - last_time_).toSec();
        last_time_ = current_time;

        //ROS_INFO_STREAM("Received sensor data");
        
        // Steuerung u: [v, omega]
        double v_cmd = u(0);        // aus /cmd_vel
        double omega_cmd = u(1);    // aus /cmd_vel
        double theta = x_(4);       // Orientierung aus vorherigem schritt

        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
        A(0,1) = dt;
        A(2,3) = dt;
        A(4,5) = dt;

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 2);
        B(0,0) = dt * cos(theta); 
        B(2,0) = dt * sin(theta);
        B(4,1) = dt;

         // Prediction
        x_ = A * x_ + B * u;			    //Zeile 2
        P_ = A * P_ * A.transpose() + R_;	//Zeile 3

        //UPDATE Zeilen 4,5 und 6
        //Messungen aus odom (keien positionen) und IMU
        double vx = odom_msg->twist.twist.linear.x;
        double vy = odom_msg->twist.twist.linear.y;
        double theta_messured = imu_msg->orientation.z;
        double omega_messured = imu_msg->angular_velocity.z;

        z(0)= vx;
        z(1)= vy;
        z(2)= theta_messured;
        z(3)= omega_messured;

        C(0, 1) = 10.0;  // vx
        C(1, 3) = 10.0;  // vy
        C(2, 4) = 0.5;  // theta
        C(3, 5) = 0.5;  // omega

        Eigen::MatrixXd S = C * P_ * C.transpose() + Q_;	    //Zeile 4 Klammer
        Eigen::MatrixXd K = P_ * C.transpose() * S.inverse();	//Zeile 4

        Eigen::VectorXd y = z - C * x_;			                //Zeile 5 Klammer
        x_ = x_ + K * y;					                    //Zeile 5

        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * C) * P_;	//Zeile 6

        //PUBLISH "Zeile 7"
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "map";

        pose_msg.pose.pose.position.x = x_(0);
        pose_msg.pose.pose.position.y = x_(2);
        pose_msg.pose.pose.position.z = 0.0; //in 2D = 0

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, x_(4));
        pose_msg.pose.pose.orientation = tf2::toMsg(q);

        for (int i = 0; i < 36; ++i)
            pose_msg.pose.covariance[i] = 0.0;

        pose_msg.pose.covariance[0]  = P_(0, 0);  // x-x
        pose_msg.pose.covariance[1]  = P_(0, 2);  // x-y
        pose_msg.pose.covariance[5]  = P_(0, 4);  // x-theta

        pose_msg.pose.covariance[6]  = P_(2, 0);  // y-x
        pose_msg.pose.covariance[7]  = P_(2, 2);  // y-y
        pose_msg.pose.covariance[11] = P_(2, 4);  // y-theta

        pose_msg.pose.covariance[30] = P_(4, 0);  // theta-x
        pose_msg.pose.covariance[31] = P_(4, 2);  // theta-y
        pose_msg.pose.covariance[35] = P_(4, 4);  // theta-theta

        pub_.publish(pose_msg);
        //ROS_INFO_STREAM("Published pose with covariance:\n" << P_);
    }
    //Von ChatGBT erstellt für die übersicht der P_ matrix
    //          |  0: x   |  1: vx  |  2: y   |  3: vy  |  4: theta |  5: omega
    // ---------|---------|---------|---------|---------|----------|----------
    // 0: x     |   0     |   1     |   2     |   3     |    4     |    5
    // 1: vx    |   6     |   7     |   8     |   9     |   10     |   11
    // 2: y     |  12     |  13     |  14     |  15     |   16     |   17
    // 3: vy    |  18     |  19     |  20     |  21     |   22     |   23
    // 4: theta |  24     |  25     |  26     |  27     |   28     |   29
    // 5: omega |  30     |  31     |  32     |  33     |   34     |   35
    //
    /*
        you can add your filter methods here
    */
   void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u(0) = msg->linear.x;
    u(1) = msg->angular.z;
    //ROS_INFO_STREAM("Received cmd_vel: v = " << u(0) << ", omega = " << u(1));
}  



    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>> sync_;
    ros::Publisher pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Time last_time_;

    // You can add your filter objects here
    // example for A:
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 2);
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(6, 6) * 0.0001;
    Eigen::MatrixXd R_ = Eigen::MatrixXd::Identity(6, 6) * 0.0001;
    Eigen::MatrixXd Q_ = Eigen::MatrixXd::Identity(4, 4) * 0.0001;
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(6);          //[x,vx,y,vy,theta,omega]
    Eigen::VectorXd u = Eigen::VectorXd::Zero(2);           //[v,omega]
    Eigen::VectorXd z = Eigen::VectorXd::Zero(4); 
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(4, 6);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_node_kalman");
    ros::NodeHandle nh("~");
    FilterNode node(nh);
    ros::spin();

    return 0;
}

/*NOTIZEN
Eigen::MatrixXd S = C * P_ * C.transpose() + Q_;        // (4×6)(6×6)(6×4) + (4×4) = (4×4)
Eigen::MatrixXd K = P_ * C.transpose() * S.inverse();   // (6×6)(6×4)(4×4)^-1 = (6×4)
Eigen::VectorXd y = z - C * x_;                         // (4×1) - (4×6)(6×1) = (4×1)
x_ = x_ + K * y;                                        // (6×1) + (6×4)(4×1) = (6×1)
P_ = (I - K*C) * P_;                                    // (6×6)(6×6) = (6×6)
*/