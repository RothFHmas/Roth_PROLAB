#!/usr/bin/env python3
import matplotlib
matplotlib.use('Agg')
import rospy
import os
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from matplotlib.patches import Ellipse

class PosePlotAccumulator:
    def __init__(self):
        rospy.init_node("pose_plot_accumulator")

        self.mode = rospy.get_param("~mode", "kf")  # kf, ekf, pf
        self.duration = rospy.get_param("~duration", 60)
        self.save_dir = rospy.get_param("~save_dir", "/tmp")
        self.output_file = rospy.get_param("~output_file", "plot.png")
        self.interval = 10.0  # Sekunden

        os.makedirs(self.save_dir, exist_ok=True)

        self.odom_trajectory = []             # Liste der Odom-Positionen
        self.predicted_positions = []         # Liste: (x, y)
        self.predicted_covariances = []       # Liste: 2x2-Cov-Matrix
        self.particles = []                   # Liste von Partikel-Sätzen: Liste[List[(x,y,theta)]]

        self.last_time = rospy.Time.now()

        rospy.Subscriber("/odom", Odometry, self.callback_odom)

        if self.mode == "kf":
            rospy.Subscriber("/prediction_KF", PoseWithCovarianceStamped, self.callback_pose_cov)
        elif self.mode == "ekf":
            rospy.Subscriber("/prediction_EKF", PoseWithCovarianceStamped, self.callback_pose_cov)
        elif self.mode == "pf":
            rospy.Subscriber("/prediction_particle", PoseArray, self.callback_pose_array)
        else:
            rospy.logerr("Unknown mode: %s", self.mode)
            return

        rospy.loginfo("PosePlotAccumulator running in mode: %s", self.mode)
        rospy.Timer(rospy.Duration(self.duration), self.finish_plot, oneshot=True)
        rospy.spin()

    def callback_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_trajectory.append((x, y))

    def callback_pose_cov(self, msg):
        now = rospy.Time.now()
        if (now - self.last_time).to_sec() < self.interval:
            return

        # Verwende Position aus der Prediction-Message (nicht Odom!)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.predicted_positions.append((x, y))

        cov = np.array(msg.pose.covariance).reshape(6, 6)[0:2, 0:2]
        self.predicted_covariances.append(cov)

        self.last_time = now

    def callback_pose_array(self, msg):
        now = rospy.Time.now()
        if (now - self.last_time).to_sec() < self.interval:
            return
        particle_list = []
        for p in msg.poses:
            x = p.position.x
            y = p.position.y
            # Orientierung aus Quaternion holen
            qz = p.orientation.z
            qw = p.orientation.w
            theta = 2 * np.arctan2(qz, qw)  # Nur yaw
            particle_list.append((x, y, theta))
        self.particles.append(particle_list)
        self.last_time = now

    def plot_covariance_ellipse(self, x, y, cov, ax, n_std=1.0, **kwargs):
        vals, vecs = np.linalg.eigh(cov)
        angle = np.degrees(np.arctan2(*vecs[:, 1][::-1]))
        width, height = 2 * n_std * np.sqrt(vals)
        ell = Ellipse((x, y), width, height, angle, fill=False, **kwargs)
        ax.add_patch(ell)

    def finish_plot(self, event):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')

        # Trajektorie aus Odom-Daten
        if len(self.odom_trajectory) > 1:
            x_odom, y_odom = zip(*self.odom_trajectory)
            ax.plot(x_odom, y_odom, 'k-', linewidth=1, label='Trajektorie (/odom)')

        # KF/ EKF: Positionen + Unsicherheit
        if self.mode in ["kf", "ekf"]:
            for (x, y), cov in zip(self.predicted_positions, self.predicted_covariances):
                ax.plot(x, y, 'bo', markersize=3)
                self.plot_covariance_ellipse(x, y, cov, ax, edgecolor='blue', linewidth=1)

        # PF: Partikel als Pfeile
        elif self.mode == "pf":
            for i, particle_set in enumerate(self.particles):
                for j, (x, y, theta) in enumerate(particle_set):
                    dx = 0.05 * np.cos(theta)
                    dy = 0.05 * np.sin(theta)
                    ax.arrow(x, y, dx, dy, head_width=0.02, head_length=0.03, fc='red', ec='red', alpha=0.5 if i < len(self.particles) - 1 else 1.0)

        ax.set_title(f"Plot-Modus: {self.mode.upper()}")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.legend()

        path = os.path.join(self.save_dir, self.output_file)
        plt.savefig(path)
        rospy.loginfo("Plot saved to: %s", path)
        plt.close()
        rospy.signal_shutdown("Fertig")

if __name__ == "__main__":
    PosePlotAccumulator()
