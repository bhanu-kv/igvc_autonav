#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Pose
from sklearn.cluster import DBSCAN
import numpy as np
import math
from nav2_simple_commander.robot_navigator import BasicNavigator

coords = []

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.coord_subs = self.create_subscription(Pose,'/destination_pose',self.detect_callback,10)
        self.path_publisher = self.create_publisher(Path, 'plan', 10)
        self.nav_publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.first_message_received = False
        self.navigator = BasicNavigator()
    def get_quaternion_from_euler(self,roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def detect_callback(self,msg):
        if not self.first_msg_received:
            msg.position.x = x
            msg.position.y = y
            msg.position.z = 0
            self.publish_trajectory([x,y])
            self.first_msg_received = True
    # def scan_callback(self, msg):
    #     if not self.first_message_received:
    #         new_coords = self.coord_change(msg.ranges, msg.angle_min, msg.angle_increment)
    #         for coord in new_coords:
    #             coords.append([coord[0], coord[1]])

    #         np_coords = np.array(coords)
    #         valid_coords = np_coords[np.isfinite(np_coords).all(axis=1)]

    #         if valid_coords.size > 0:
    #             eps = 0.5
    #             min_samples = 5
    #             dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    #             clusters = dbscan.fit_predict(valid_coords)

    #             unique_clusters = set(clusters)
    #             centroids = []
    #             for cluster_id in unique_clusters:
    #                 if cluster_id != -1:
    #                     cluster_points = valid_coords[clusters == cluster_id]
    #                     centroid = np.mean(cluster_points, axis=0)
    #                     if (centroid[0] > 0 and centroid[1] < 0):
    #                         centroids.append(centroid)
    #                     self.get_logger().info(f"Cluster {cluster_id}: Centroid - {centroid}")

    #         if centroids:
    #             self.new_centroids = {}
    #             for i in centroids:
    #                 x, y = i[0], i[1]
    #                 self.new_centroids[x**2 + y**2] = i
    #             l = list(self.new_centroids.keys())
    #             l.sort()
    #             centroids = [self.new_centroids[l[0]], self.new_centroids[l[1]]-0.35]
    #             self.mean_centroid = 0.80*centroids[1] + 0.20*centroids[0]
    #             self.final_parallel = 0.9*centroids[1] + 0.1*centroids[0] 
    #             self.final_parallel1 = 0.5*centroids[1] + 0.5*centroids[0] 
    #             self.get_logger().info(f"Mean Centroid: {self.mean_centroid}")
    #             self.publish_trajectory(self.mean_centroid,centroids)

    #         self.first_message_received = True

    # def coord_change(self, ranges, angle_min, angle_increment):
    #     new_coords = []
    #     angle = angle_min
    #     for r in ranges:
    #         if not math.isfinite(r) or r == 0.0:
    #             continue
    #         #if r <= 5:
    #         x = r * math.cos(angle)
    #         y = r * math.sin(angle)
    #         new_coords.append((x, y))
    #         angle += angle_increment
    #         # else:
    #         #     angle += angle_increment
    #     return new_coords

    def publish_trajectory(self, mean_centroid): #,centroids):
        init_pos = [0.0, 0.0]
        fin_pos = mean_centroid

        th = math.atan2(fin_pos[1] - init_pos[1], fin_pos[0] - init_pos[0])
        th = 1.57 - th
        th = round(th, 2)
        dist = math.dist(init_pos, fin_pos)
        r = (dist / 4) / math.cos(th)

        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        path_poses = []

        for t in np.arange(-3.14, 2 * (th - 3.14), 0.01):
            x = init_pos[0] + r * math.sin(t)
            y = init_pos[1] + r + r * math.cos(t)
            z = 0.0

            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            roll = 0.0
            pitch = 0.0
            yaw = np.round(math.atan((math.sin(t)-math.sin(t-0.01))/(math.cos(t)-math.cos(t-0.01))))
            qx, qy, qz, qw = self.get_quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation.w = qw
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            #pose.pose.orientation.w = math.atan((math.sin(t)-math.sin(t-0.01))/(math.cos(t)-math.cos(t-0.01))) #1.0 #math.atan(y/x)
            path_msg.poses.append(pose)
            path_poses.append(pose)

        for t in np.arange(4.71 - 2 * th, 1.57, 0.01):
            x = fin_pos[0] + r * math.cos(t)
            y = fin_pos[1] - r + r * math.sin(t)
            z = 0.0

            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            roll = 0.0
            pitch = 0.0
            yaw = (math.atan((math.sin(t)-math.sin(t-0.01))/(math.cos(t)-math.cos(t-0.01))))
            qx, qy, qz, qw = self.get_quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation.w = qw
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            path_msg.poses.append(pose)
            path_poses.append(pose)
        w = pose.pose.orientation.w
        # for t in np.arange(np.round(fin_pos[0],4), np.round(self.final_parallel[0],4), 0.01):
        #     pose = PoseStamped()
        #     pose.header.frame_id = "base_link"
        #     pose.header.stamp = self.get_clock().now().to_msg()
        #     pose.pose.position.y = self.final_parallel[1]
        #     pose.pose.position.x = t
        #     pose.pose.position.z = 0.0
        #     qx, qy, qz, qw = self.get_quaternion_from_euler(0.0,0.0,math.atan((centroids[1][1]-centroids[0][1] )/ (centroids[1][0]-centroids[0][0])))
        #     pose.pose.orientation.w = qw
        #     pose.pose.orientation.x = qx
        #     pose.pose.orientation.y = qy
        #     pose.pose.orientation.z = qz
        #     #pose.pose.orientation.w = math.atan((centroids[1][1]-centroids[0][1] )/ (centroids[1][0]-centroids[0][0]))#math.atan(0)#1.0#math.atan(y/x)
        #     path_msg.poses.append(pose)
        #     path_poses.append(pose)              

        self.path_publisher.publish(path_msg)
        self.get_logger().info('Published a new trajectory.')
        #self.navigator.followPath(path_msg)
        self.navigator.goThroughPoses(path_poses)
        while not self.navigator.isTaskComplete():
            print("navigating")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


