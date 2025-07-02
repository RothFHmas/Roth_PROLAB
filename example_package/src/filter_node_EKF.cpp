#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
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

        pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/prediction_EKF", 10);

        // Initialisierung der Filtergrößen
        x_ = Eigen::VectorXd::Zero(6); // [x, y, theta, vx, vy, omega]
        P_ = Eigen::MatrixXd::Identity(6, 6) * 0.2;
        Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.02;
        R_ = Eigen::MatrixXd::Identity(6, 6) * 0.02;
        H_ = Eigen::MatrixXd::Identity(6, 6); // Direkte Messung von theta, vx, vy, omega
        last_time_ = ros::Time(0);
        
    }

private:
    void sensorCallback(const nav_msgs::Odometry::ConstPtr &odom_msg, const sensor_msgs::Imu::ConstPtr &imu_msg)
    {
        ros::Time current_time = odom_msg->header.stamp;
        double dt = (last_time_.isZero()) ? 0.01 : (current_time - last_time_).toSec();
        last_time_ = current_time;

        // Steuerung u: [v, omega]
        //double v = odom_msg->twist.twist.linear.x;
        //double omega = imu_msg->angular_velocity.z;
         double v = u(0);        // aus /cmd_vel
        double omega = u(1);    // aus /cmd_vel
        double theta = x_(2);

        Eigen::Matrix<double, 2, 1> u;
        u << v, omega;

        //EKF: PREDICTION 
        // g(x,u)
        Eigen::VectorXd x_pred = x_;
        x_pred(0) += dt * v * cos(theta);   // x
        x_pred(1) += dt * v * sin(theta);   // y
        x_pred(2) += dt * omega;            // theta
        x_pred(3) = v * cos(theta);         // vx
        x_pred(4) = v * sin(theta);         // vy
        x_pred(5) = omega;                  // omega

        // F: System-Jacobi von g(x,u)
        Eigen::MatrixXd G = Eigen::MatrixXd::Identity(6,6);
        G(0,2) = -dt * v * sin(theta);  // d(g1)/d(theta)
        G(1,2) = dt * v * cos(theta);   // d(g2)/d(theta)

        // G: Steuer-Jacobi (optional, falls du B brauchst)
        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(6,2);
        F(0,0) = dt * cos(theta);
        F(1,0) = dt * sin(theta);
        F(2,1) = dt;
        F(3,0) = cos(theta);
        F(4,0) = sin(theta);
        F(5,1) = 1.0;

        x_ = x_pred;                        //Zeile 2
        P_ = G * P_ * G.transpose() + Q_;   //Zeile 3

        //EKF: UPDATE
        double measured_theta = tf2::getYaw(imu_msg->orientation);  // aus IMU
        double measured_vx = odom_msg->twist.twist.linear.x;        // aus Odometrie
        double measured_vy = odom_msg->twist.twist.linear.y;        // aus Odometrie
        double measured_omega = imu_msg->angular_velocity.z;        // aus IMU
        
        // z = Messvektor
        Eigen::VectorXd z(6);
        z << x_(0),
            x_(1),
            measured_theta,
            measured_vx,
            measured_vy,
            measured_omega;

        // h(x): erwartete Messung
        Eigen::VectorXd z_pred(6);
        z_pred << x_(0), x_(1), x_(2), x_(3), x_(4), x_(5);

        // H: Jacobi-Matrix der Messfunktion h(x)
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(6,6);

        // Kalman-Gain
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // Update
        Eigen::VectorXd y = z - z_pred;
        // Winkel-Differenz normieren
        y(2) = atan2(sin(y(2)), cos(y(2)));

        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(6,6) - K * H) * P_;

        // ======= PUBLISH =======
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = x_(0);
        pose_msg.pose.pose.position.y = x_(1);
        pose_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, x_(2));
        pose_msg.pose.pose.orientation = tf2::toMsg(q);

        // Setze alles auf 0.0
        for (int i = 0; i < 36; ++i)
        pose_msg.pose.covariance[i] = 0.0;

        // Setze nur die relevanten Einträge für x, y und yaw (theta)
        pose_msg.pose.covariance[0] = P_(0, 0);  // x-x
        pose_msg.pose.covariance[1] = P_(0, 1);  // x-y
        pose_msg.pose.covariance[6] = P_(1, 0);  // y-x
        pose_msg.pose.covariance[7] = P_(1, 1);  // y-y
        pose_msg.pose.covariance[35] = P_(2, 2); // yaw-yaw

        pub_.publish(pose_msg);
    }
/* von ChatGBT erstellt, um relavante einträge besser finden zu können
    |   x     y     z   roll  pitch  yaw
    -----------------------------------------------
    x      --> |  c[0]  c[1]  c[2]  c[3]  c[4]  c[5]
    y      --> |  c[6]  c[7]  c[8]  c[9]  c[10] c[11]
    z      --> |  c[12] c[13] c[14] c[15] c[16] c[17]
    roll   --> |  c[18] c[19] c[20] c[21] c[22] c[23]
    pitch  --> |  c[24] c[25] c[26] c[27] c[28] c[29]
    yaw    --> |  c[30] c[31] c[32] c[33] c[34] c[35]
*/
   void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u(0) = msg->linear.x;
    u(1) = msg->angular.z;
    //ROS_INFO_STREAM("Received cmd_vel: v = " << u(0) << ", omega = " << u(1));
} 
    // ROS-Komponenten
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>> sync_;
    ros::Publisher pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Time last_time_;

    // Filterzustand
    Eigen::VectorXd x_;     // [x, y, theta, vx, vy, omega]
    Eigen::MatrixXd P_;     // Kovarianzmatrix
    Eigen::MatrixXd Q_;     // Prozessrauschen
    Eigen::MatrixXd R_;     // Messrauschen
    Eigen::MatrixXd H_;     // Messmatrix
    Eigen::VectorXd u = Eigen::VectorXd::Zero(2);           //[v,omega]
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalman_filter_node");
    ros::NodeHandle nh("~");
    FilterNode node(nh);
    ros::spin();
    return 0;
}