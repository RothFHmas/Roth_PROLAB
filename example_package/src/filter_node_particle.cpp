#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <random>
#include <algorithm>

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <example_package/ParticleFilterConfig.h>

class ParticleFilterNode
{
public:
    ParticleFilterNode(ros::NodeHandle& nh)
        : gen_(rd_()), initialized_(false)
    {
        ros::NodeHandle pnh("~");
        pnh.param("num_particles", num_particles_, 100);
        pnh.param("alpha1", alpha1_, 0.01);
        pnh.param("alpha2", alpha2_, 0.01);
        pnh.param("alpha3", alpha3_, 0.01);
        pnh.param("alpha4", alpha4_, 0.01);

        pnh.param("init_std_x", init_std_x_, 0.3);
        pnh.param("init_std_y", init_std_y_, 0.5);
        pnh.param("init_std_theta", init_std_theta_, 0.5);

        pnh.param("outlier_distance_threshold", outlier_distance_threshold, 2.0); // default 2 Meter


        particles_.resize(num_particles_, Eigen::Vector3d::Zero());
        weights_.resize(num_particles_, 1.0 / num_particles_);

        odom_sub_.subscribe(nh, "/odom", 10);
        imu_sub_.subscribe(nh, "/imu", 10);

        sync_ = std::make_shared<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>>(odom_sub_, imu_sub_, 10);
        sync_->registerCallback(boost::bind(&ParticleFilterNode::sensorCallback, this, _1, _2));

        pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("/prediction_particle", 10);

        dynamic_reconfigure::Server<example_package::ParticleFilterConfig>::CallbackType f;
        f = boost::bind(&ParticleFilterNode::reconfigureCallback, this, _1, _2);
        dr_srv_.setCallback(f);
    }

private:
    void sensorCallback(const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::Imu::ConstPtr& imu_msg)
    {
        ros::Time current_time = odom_msg->header.stamp;
        double dt = (last_time_.isZero()) ? 0.01 : (current_time - last_time_).toSec();
        last_time_ = current_time;

        double v = odom_msg->twist.twist.linear.x;
        double omega = imu_msg->angular_velocity.z;

        if (!initialized_)
        {
            std::normal_distribution<double> dist_x(0.0, init_std_x_);
            std::normal_distribution<double> dist_y(0.0, init_std_y_);
            std::normal_distribution<double> dist_theta(0.0, init_std_theta_);

            for (auto& particle : particles_)
            {
                particle(0) = dist_x(gen_);
                particle(1) = dist_y(gen_);
                particle(2) = dist_theta(gen_);
            }
            initialized_ = true;
            std::fill(weights_.begin(), weights_.end(), 1.0 / num_particles_);
            return;
        }

        // Motion update
        std::normal_distribution<double> noise_v(0.0, sqrt(alpha1_ * v * v + alpha2_ * omega * omega));
        std::normal_distribution<double> noise_omega(0.0, sqrt(alpha3_ * v * v + alpha4_ * omega * omega));

        for (auto& particle : particles_)
        {
            double theta = particle(2);
            double noisy_v = v + noise_v(gen_);
            double noisy_omega = omega + noise_omega(gen_);

            particle(0) += dt * noisy_v * cos(theta);
            particle(1) += dt * noisy_v * sin(theta);
            particle(2) += dt * noisy_omega;
            particle(2) = atan2(sin(particle(2)), cos(particle(2)));
        }

        // Gewichtung basierend auf omega UND v
        double sigma_omega = 0.5; // Sensorrauschen für omega
        double sigma_v = 0.1;     // Sensorrauschen für v
        weights_.clear();
        double total_weight = 0.0;

        for (const auto& particle : particles_)
        {
            
            double predicted_omega = imu_msg->angular_velocity.z;
            double predicted_v = odom_msg->twist.twist.linear.x; 

            // Differenzen
            double diff_omega = omega - predicted_omega;
            double diff_v = v - predicted_v;

            // Kombinierte Gewichtung (Produkt von Gaußschen Wahrscheinlichkeiten)
            double w_omega = exp(- (diff_omega * diff_omega) / (2 * sigma_omega * sigma_omega));
            double w_v = exp(- (diff_v * diff_v) / (2 * sigma_v * sigma_v));
            double weight = w_omega * w_v;

            weights_.push_back(weight);
            total_weight += weight;
        }

        // Normiere
        for (auto& w : weights_) w /= (total_weight + 1e-8);

        // Schlechteste 1 % entfernen
        int num_to_remove = static_cast<int>(num_particles_ * 0.01);
        std::vector<size_t> indices(weights_.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(), [&](size_t i1, size_t i2) {
            return weights_[i1] < weights_[i2];
        });

        std::vector<Eigen::Vector3d> surviving_particles;
        std::vector<double> surviving_weights;
        for (size_t i = num_to_remove; i < indices.size(); ++i)
        {
            surviving_particles.push_back(particles_[indices[i]]);
            surviving_weights.push_back(weights_[indices[i]]);
        }

        // Resampling aus verbleibenden Partikeln
        int num_to_sample = num_particles_ - static_cast<int>(surviving_particles.size());
        std::discrete_distribution<> dist(surviving_weights.begin(), surviving_weights.end());

        std::normal_distribution<double> noise_x(0.0, 0.1);
        std::normal_distribution<double> noise_y(0.0, 0.1);
        std::normal_distribution<double> noise_theta(0.0, 0.1);

        // Basis für Resampling (surviving_particles)
        std::vector<Eigen::Vector3d> base_particles = surviving_particles;
        std::uniform_int_distribution<> base_dist(0, base_particles.size() - 1);

        while (surviving_particles.size() < num_particles_)
        {
            Eigen::Vector3d base = base_particles[base_dist(gen_)];
            base(0) += noise_x(gen_);
            base(1) += noise_y(gen_);
            base(2) += noise_theta(gen_);
            base(2) = atan2(sin(base(2)), cos(base(2)));
            surviving_particles.push_back(base);
        }

        particles_ = surviving_particles;
        weights_.assign(num_particles_, 1.0 / num_particles_);

        // Schwellenwert initialisieren (z.B. 2.0 Meter)
        double outlier_distance_threshold_ = 2.0;

        // --- Entferne stärkste 5 Ausreißer (nach Abstand zum Mittelwert) nur wenn Distanz > Schwellenwert 
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const auto& p : particles_) mean += p;
        mean /= particles_.size();

        // Berechne Abstände
        std::vector<std::pair<size_t, double>> distances;
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            double dist = (particles_[i].head<2>() - mean.head<2>()).norm(); // nur x,y
            distances.emplace_back(i, dist);
        }

        // Sortiere nach Distanz absteigend
        std::sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) {
            return a.second > b.second;
        });

        // Entferne nur jene Ausreißer, die größer als Threshold sind
        int removed_count = 0;
        std::vector<Eigen::Vector3d> filtered_particles;
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            bool is_outlier = false;
            for (int j = 0; j < 5 && j < (int)distances.size(); ++j)
            {
                if (i == distances[j].first && distances[j].second > outlier_distance_threshold_)
                {
                    is_outlier = true;
                    ++removed_count;
                    break;
                }
            }
            if (!is_outlier)
                filtered_particles.push_back(particles_[i]);
        }

        // Falls zu wenige Partikel bleiben, wieder auffüllen
        while (filtered_particles.size() < num_particles_)
        {
            Eigen::Vector3d base = base_particles[base_dist(gen_)];
            base(0) += noise_x(gen_);
            base(1) += noise_y(gen_);
            base(2) += noise_theta(gen_);
            base(2) = atan2(sin(base(2)), cos(base(2)));
            filtered_particles.push_back(base);
        }

        particles_ = filtered_particles;
        weights_.assign(num_particles_, 1.0 / num_particles_);


        // Finalisiere
        particles_ = filtered_particles;
        weights_.assign(num_particles_, 1.0 / num_particles_);

        // Ausgabe
        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = current_time;
        pose_array.header.frame_id = "odom";

        for (const auto& particle : particles_)
        {
            geometry_msgs::Pose pose;
            pose.position.x = particle(0);
            pose.position.y = particle(1);
            pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, particle(2));
            pose.orientation = tf2::toMsg(q);
            pose_array.poses.push_back(pose);
        }

        pose_pub_.publish(pose_array);
    }

    void reconfigureCallback(example_package::ParticleFilterConfig &config, uint32_t level)
    {
        alpha1_ = config.alpha1;
        alpha2_ = config.alpha2;
        alpha3_ = config.alpha3;
        alpha4_ = config.alpha4;
    }

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>> sync_;
    ros::Publisher pose_pub_;
    ros::Time last_time_;

    std::vector<Eigen::Vector3d> particles_;
    std::vector<double> weights_;
    int num_particles_;
    bool initialized_;

    double alpha1_, alpha2_, alpha3_, alpha4_;
    double init_std_x_, init_std_y_, init_std_theta_, outlier_distance_threshold;

    std::random_device rd_;
    std::mt19937 gen_;

    dynamic_reconfigure::Server<example_package::ParticleFilterConfig> dr_srv_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "particle_filter_node");
    ros::NodeHandle nh;
    ParticleFilterNode node(nh);
    ros::spin();
    return 0;
}
