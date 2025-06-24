#!/usr/bin/python3

import rospy
import utm
import math
import tf2_ros

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, PoseWithCovarianceStamped

from utm_ros.srv import LLAToUTM, LLAToUTMResponse, UTMToLLA, UTMToLLAResponse

class GNSSUTMNode:
    def __init__(self):
        rospy.init_node('utm_node')

        self.param_base_link = rospy.get_param('~base_frame', "base_link")
        self.param_local_link = rospy.get_param('~local_frame', "local")
        self.param_world_link = rospy.get_param('~world_frame', "world")
        self.param_publish_tf = rospy.get_param('~publish_tf', True)
        self.param_publish_odom = rospy.get_param('~publish_odom', True)
        self.param_use_imu = rospy.get_param('~use_imu', False)

        self.param_ignore_imu_cov = rospy.get_param('~ignore_imu_covariance', True)
        self.param_ignore_status = rospy.get_param('~ignore_fix_status', False)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        if self.param_use_imu:
            self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        self.gps_sub = rospy.Subscriber('/fix', NavSatFix, self.gps_callback)

        self.gnss_origin_pub = rospy.Publisher('/gnss/local_origin_fix', NavSatFix, queue_size=10, latch=True)
        self.pose_pub = rospy.Publisher('/gnss/pose', PoseWithCovarianceStamped, queue_size=10)

        if self.param_publish_odom:
            self.odom_pub = rospy.Publisher('/gnss/odom', Odometry, queue_size=10)

        self.lla_to_utm_srv = rospy.Service('/gnss/lla_to_utm_local', LLAToUTM, self.handle_lla_to_utm)
        self.utm_to_lla_srv = rospy.Service('/gnss/utm_local_to_lla', UTMToLLA, self.handle_utm_to_lla)

        self.orientation = Quaternion(0,0,0,1)
        self.gps_data = None

        self.zone_letter = None
        self.zone_number = None

        self.origin_x = None
        self.origin_y = None
        self.origin_z = None
        self.origin_fix = None

        print("GNSS UTM node started!")

    def handle_lla_to_utm(self, req):
        try:

            if self.zone_number is None or self.origin_x is None:
                return LLAToUTMResponse(0.0, 0.0, False)

            x, y, _, _ = utm.from_latlon(
                req.latitude, req.longitude,
                force_zone_number=self.zone_number if self.zone_number else None,
                force_zone_letter=self.zone_letter if self.zone_letter else None
            )
            
            x -= self.origin_x
            y -= self.origin_y

            return LLAToUTMResponse(x, y, True)

        except Exception as e:
            rospy.logwarn(f"LLAToUTM conversion failed: {e}")
            return LLAToUTMResponse(0.0, 0.0, False)

    def handle_utm_to_lla(self, req):
        try:

            if self.zone_number is None or self.origin_x is None:
                return UTMToLLAResponse(0.0, 0.0, 0.0, False)

            lat, lon = utm.to_latlon(req.x + self.origin_x, req.y + self.origin_y, self.zone_number, self.zone_letter)
            return UTMToLLAResponse(lat, lon, True)

        except Exception as e:
            rospy.logwarn(f"UTMToLLA conversion failed: {e}")
            return UTMToLLAResponse(0.0, 0.0, False)

    def imu_callback(self, msg):
        if not self.param_ignore_imu_cov and msg.orientation_covariance[8] > 100:
            return

        self.orientation = msg.orientation

    def gps_callback(self, msg):

        #because mavros is dumb
        if self.param_ignore_status:
            msg.status.status = 0 

        if msg.status.status == -1:
            return
        
        if msg.latitude == 0 and msg.longitude == 0:
            #probably still invalid
            return
        
        self.gps_data = msg
        self.process_data()

    def process_data(self):
        if self.gps_data is None:
            return
        
        if self.zone_letter is None:
            _, _, self.zone_number, self.zone_letter = utm.from_latlon(self.gps_data.latitude, self.gps_data.longitude)

        x, y, _, _ = utm.from_latlon(
            self.gps_data.latitude, 
            self.gps_data.longitude,
            force_zone_letter=self.zone_letter,
            force_zone_number=self.zone_number
        )
        z = self.gps_data.altitude

        cov = [0.0] * 36
        cov[0] = math.sqrt(math.fabs(self.gps_data.position_covariance[0]))
        cov[7] = math.sqrt(math.fabs(self.gps_data.position_covariance[4]))
        cov[14] = math.sqrt(math.fabs(self.gps_data.position_covariance[8]))

        #publish local origin link and fix in that location
        if self.origin_x is None:
            self.origin_x = x
            self.origin_y = y
            self.origin_z = z
            self.origin_fix = self.gps_data
            self.origin_fix.header.frame_id = self.param_local_link
            self.gnss_origin_pub.publish(self.origin_fix)

            world_to_local_msg = TransformStamped()
            world_to_local_msg.header.stamp = rospy.Time.now()
            world_to_local_msg.header.frame_id = self.param_world_link
            world_to_local_msg.child_frame_id = self.param_local_link
            world_to_local_msg.transform.translation.x = self.origin_x
            world_to_local_msg.transform.translation.y = self.origin_y
            world_to_local_msg.transform.translation.z = 0.0
            world_to_local_msg.transform.rotation.w = 1.0
            self.static_tf_broadcaster.sendTransform(world_to_local_msg)

        local_x = x - self.origin_x
        local_y = y - self.origin_y

        if self.param_publish_tf:
            local_to_base_msg = TransformStamped()
            local_to_base_msg.header.stamp = rospy.Time.now()
            local_to_base_msg.header.frame_id = self.param_local_link
            local_to_base_msg.child_frame_id = self.param_base_link
            local_to_base_msg.transform.translation.x = local_x
            local_to_base_msg.transform.translation.y = local_y
            local_to_base_msg.transform.translation.z = 0.0
            local_to_base_msg.transform.rotation = self.orientation
            self.tf_broadcaster.sendTransform(local_to_base_msg)

        if self.param_publish_odom:
            odom_msg = Odometry()
            odom_msg.header.frame_id = self.param_local_link
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.pose.pose.position.x = local_x
            odom_msg.pose.pose.position.y = local_y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = self.orientation
            odom_msg.pose.covariance = cov
            odom_msg.twist.twist.linear.x = 0
            odom_msg.twist.twist.linear.y = 0
            odom_msg.twist.twist.linear.z = 0
            odom_msg.twist.twist.angular.x = 0
            odom_msg.twist.twist.angular.y = 0
            odom_msg.twist.twist.angular.z = 0
            self.odom_pub.publish(odom_msg)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = self.param_local_link
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = local_x
        pose_msg.pose.pose.position.y = local_y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation = self.orientation
        pose_msg.pose.covariance = cov
        self.pose_pub.publish(pose_msg)

node = GNSSUTMNode()
rospy.spin()
