#!/usr/bin/python3

import rospy
import math
import tf2_ros
from pyproj import CRS, Transformer, Proj

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, PoseWithCovarianceStamped

from utm_ros.srv import LLAToENU, LLAToENUResponse, ENUToLLA, ENUToLLAResponse

import copy

def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0  
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c * 1000.0

class GNSSENUNode:
    def __init__(self):
        rospy.init_node('enu_node')
        self.param_ellipsoid_model = rospy.get_param('~ellipsoid_model', "WGS84")

        self.param_base_link = rospy.get_param('~base_frame', "base_link")
        self.param_local_link = rospy.get_param('~local_frame', "local")
        self.param_publish_tf = rospy.get_param('~publish_tf', True)
        self.param_publish_odom = rospy.get_param('~publish_odom', True)
        self.param_use_imu = rospy.get_param('~use_imu', False)

        self.param_ignore_imu_cov = rospy.get_param('~ignore_imu_covariance', True)
        self.param_ignore_status = rospy.get_param('~ignore_fix_status', False)

        # Consistent origin parameters
        self.param_origin_lat = rospy.get_param('~origin_latitude', None)
        self.param_origin_lon = rospy.get_param('~origin_longitude', None)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.orientation = Quaternion(0,0,0,1)

        self.local_origin_fix = None
        self.gps_fix = None

        self.enu_proj = None
        self.proj_lla_to_enu = None
        self.proj_enu_to_lla = None

        if self.param_use_imu:
            self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        self.gps_sub = rospy.Subscriber('/gnss/fix', NavSatFix, self.gps_callback)

        self.local_origin_fix_pub = rospy.Publisher('/gnss/local_origin_fix', NavSatFix, queue_size=10, latch=True)
        self.pose_pub = rospy.Publisher('/gnss/pose', PoseWithCovarianceStamped, queue_size=10)

        if self.param_publish_odom:
            self.odom_pub = rospy.Publisher('/gnss/odom', Odometry, queue_size=10)

        self.lla_to_enu_srv = rospy.Service('/gnss/lla_to_enu', LLAToENU, self.handle_lla_to_enu)
        self.enu_to_lla_srv = rospy.Service('/gnss/enu_to_lla', ENUToLLA, self.handle_enu_to_lla)

        rospy.loginfo("GNSS to ENU node ready.")

    def convert_lla_to_enu(self, lat, lon):
        try:
            east, north = self.proj_lla_to_enu.transform(lon, lat)
            return east, north
        except Exception as e:
            rospy.logerr(f"LLA to ENU conversion failed: {e}")
            return None, None

    def convert_enu_to_lla(self, east, north):
        try:
            lon, lat = self.proj_enu_to_lla.transform(east, north)
            return lat, lon
        except Exception as e:
            rospy.logerr(f"ENU to LLA conversion failed: {e}")
            return None, None

    def handle_lla_to_enu(self, req):
        try:
            east, north = self.convert_lla_to_enu(req.latitude, req.longitude)
            if east is None or north is None:
                return LLAToENUResponse(0.0, 0.0, False)
            return LLAToENUResponse(east, north, True)
        except Exception as e:
            rospy.logerr(f"LLA to ENU service conversion failed: {e}")
            return LLAToENUResponse(0.0, 0.0, False)

    def handle_enu_to_lla(self, req):
        try:
            lat, lon = self.convert_enu_to_lla(req.x, req.y)
            if lat is None or lon is None:
                return ENUToLLAResponse(0.0, 0.0, False)
            return ENUToLLAResponse(lat, lon, True)
        except Exception as e:
            rospy.logwarn(f"ENU to LLA service conversion failed: {e}")
            return ENUToLLAResponse(0.0, 0.0, False)

    def imu_callback(self, msg):
        if not self.param_ignore_imu_cov and msg.orientation_covariance[8] > 100:
            return

        self.orientation = msg.orientation

    def gps_callback(self, msg):
        # because mavros is dumb
        if self.param_ignore_status:
            msg.status.status = 0 

        if msg.status.status == -1:
            rospy.logwarn(f"GNSS status invalid, ignoring.")
            return
        
        if msg.latitude == 0 and msg.longitude == 0:
            rospy.logwarn(f"GNSS coords zero, ignoring.")
            # probably still invalid
            return
        
        if self.gps_fix is None:
            self.init_gps(copy.deepcopy(msg))
        
        self.gps_fix = msg
        self.update()

    def init_gps(self, msg):
        self.local_origin_fix = msg
        self.local_origin_fix.header.frame_id = self.param_local_link

        # Use consistent origin if specified
        if self.param_origin_lat is not None and self.param_origin_lon is not None:
            dist = int(haversine(
                self.param_origin_lat, 
                self.param_origin_lon,
                self.local_origin_fix.latitude,
                self.local_origin_fix.longitude
            ))

            if dist < 20_000:
                self.local_origin_fix.latitude = self.param_origin_lat
                self.local_origin_fix.longitude = self.param_origin_lon
                rospy.loginfo(f"GNSS Origin override enabled, distance {int(dist)}m!")
            else:
                rospy.logwarn(f"GNSS Origin override over 20 km away, ignoring to maintain precision.")
        
        proj_string = "+proj=tmerc "
        proj_string += f"+lat_0={self.local_origin_fix.latitude} "
        proj_string += f"+lon_0={self.local_origin_fix.longitude} "
        proj_string += f"+k=1 +x_0=0 +y_0=0 +ellps={self.param_ellipsoid_model} +units=m +no_defs +type=crs"
        world_enu_proj = Proj(proj_string)

        self.proj_lla_to_enu = Transformer.from_crs("EPSG:4326", world_enu_proj.crs, always_xy=True)
        self.proj_enu_to_lla = Transformer.from_crs(world_enu_proj.crs, "EPSG:4326", always_xy=True)

        rospy.loginfo(f"GNSS Initialized local origin at {self.local_origin_fix.latitude}, {self.local_origin_fix.longitude}")                

    def update(self):
        east, north = self.convert_lla_to_enu(
            self.gps_fix.latitude, 
            self.gps_fix.longitude
        )
        
        if east is None or north is None:
            rospy.logwarn("Failed to convert GPS position to world ENU")
            return

        # Keep local origin fix up to date
        self.local_origin_fix.header.stamp = rospy.Time.now()
        self.local_origin_fix_pub.publish(self.local_origin_fix)

        if self.param_publish_tf:
            local_to_base_msg = TransformStamped()
            local_to_base_msg.header.stamp = rospy.Time.now()
            local_to_base_msg.header.frame_id = self.param_local_link
            local_to_base_msg.child_frame_id = self.param_base_link
            local_to_base_msg.transform.translation.x = east
            local_to_base_msg.transform.translation.y = north
            local_to_base_msg.transform.translation.z = 0.0
            local_to_base_msg.transform.rotation = self.orientation
            self.tf_broadcaster.sendTransform(local_to_base_msg)

        cov = [0.0] * 36
        cov[0] = math.sqrt(math.fabs(self.gps_fix.position_covariance[0]))
        cov[7] = math.sqrt(math.fabs(self.gps_fix.position_covariance[4]))
        cov[14] = math.sqrt(math.fabs(self.gps_fix.position_covariance[8]))

        if self.param_publish_odom:
            odom_msg = Odometry()
            odom_msg.child_frame_id = self.param_base_link 
            odom_msg.header.frame_id = self.param_local_link
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.pose.pose.position.x = east
            odom_msg.pose.pose.position.y = north
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
        pose_msg.pose.pose.position.x = east
        pose_msg.pose.pose.position.y = north
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation = self.orientation
        pose_msg.pose.covariance = cov
        self.pose_pub.publish(pose_msg)

node = GNSSENUNode()
rospy.spin()