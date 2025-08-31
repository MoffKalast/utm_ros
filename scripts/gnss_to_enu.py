#!/usr/bin/python3

import rospy
import math
import tf2_ros
from pyproj import Proj, Transformer

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, PoseWithCovarianceStamped

from utm_ros.srv import LLAToUTM, LLAToUTMResponse, UTMToLLA, UTMToLLAResponse

class GNSSENUNode:
    def __init__(self):
        rospy.init_node('enu_node')

        self.param_base_link = rospy.get_param('~base_frame', "base_link")
        self.param_local_link = rospy.get_param('~local_frame', "local")
        self.param_world_link = rospy.get_param('~world_frame', "world")
        self.param_publish_tf = rospy.get_param('~publish_tf', True)
        self.param_publish_odom = rospy.get_param('~publish_odom', True)
        self.param_use_imu = rospy.get_param('~use_imu', False)

        self.param_ignore_imu_cov = rospy.get_param('~ignore_imu_covariance', True)
        self.param_ignore_status = rospy.get_param('~ignore_fix_status', False)

        # World origin parameters
        self.param_world_origin_lat = rospy.get_param('~world_origin_latitude', None)
        self.param_world_origin_lon = rospy.get_param('~world_origin_longitude', None)
        self.param_world_altitude = rospy.get_param('~world_altitude', 0.0)

        # Auto world origin parameters
        self.param_auto_world_origin = rospy.get_param('~auto_world_origin', False)
        self.param_world_origin_precision = rospy.get_param('~world_origin_precision', 0.05)  # ~5km radius

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        if self.param_use_imu:
            self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        self.gps_sub = rospy.Subscriber('/fix', NavSatFix, self.gps_callback)

        self.gnss_origin_pub = rospy.Publisher('/gnss/local_origin_fix', NavSatFix, queue_size=10, latch=True)
        self.world_origin_pub = rospy.Publisher('/gnss/world_origin_fix', NavSatFix, queue_size=10, latch=True)
        self.pose_pub = rospy.Publisher('/gnss/pose', PoseWithCovarianceStamped, queue_size=10)

        if self.param_publish_odom:
            self.odom_pub = rospy.Publisher('/gnss/odom', Odometry, queue_size=10)

        # Keep service names consistent for compatibility
        self.lla_to_enu_srv = rospy.Service('/gnss/lla_to_utm_local', LLAToUTM, self.handle_lla_to_enu)
        self.enu_to_lla_srv = rospy.Service('/gnss/utm_local_to_lla', UTMToLLA, self.handle_enu_to_lla)

        self.orientation = Quaternion(0,0,0,1)
        self.gps_data = None

        # ENU projection variables
        self.world_enu_proj = None
        self.world_wgs84_to_enu = None
        self.world_enu_to_wgs84 = None
        
        # World origin (set via parameters or auto-calculated)
        self.world_origin_lat = self.param_world_origin_lat
        self.world_origin_lon = self.param_world_origin_lon
        self.world_origin_alt = self.param_world_altitude
        
        # Local origin (first GPS fix)
        self.local_origin_lat = None
        self.local_origin_lon = None
        self.local_origin_alt = None
        self.local_origin_fix = None
        
        # World->Local transform (computed once when local origin is established)
        self.world_to_local_x = None
        self.world_to_local_y = None
        
        # Setup world ENU projection if parameters are provided
        if self.world_origin_lat is not None and self.world_origin_lon is not None:
            self.setup_world_enu_projection()
            self.publish_world_origin()
        else:
            if self.param_auto_world_origin:
                rospy.loginfo(f"Auto world origin enabled with precision {self.param_world_origin_precision}° (~{self.param_world_origin_precision * 111:.1f}km)")
            else:
                rospy.logwarn("World origin not set via parameters. Will use first GPS fix as both world and local origin.")

        rospy.loginfo("GNSS ENU node started!")
        if self.world_origin_lat is not None:
            rospy.loginfo(f"World origin: {self.world_origin_lat:.6f}, {self.world_origin_lon:.6f}, alt={self.world_origin_alt:.1f}m")

    def round_to_grid(self, lat, lon, precision):
        """Round lat/lon to nearest grid point for consistent world origins"""
        rounded_lat = round(lat / precision) * precision
        rounded_lon = round(lon / precision) * precision
        return rounded_lat, rounded_lon

    def setup_world_enu_projection(self):
        """Setup the ENU projection centered at the world origin"""
        proj_string = f"+proj=tmerc +lat_0={self.world_origin_lat} +lon_0={self.world_origin_lon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"
        self.world_enu_proj = Proj(proj_string)
        
        # Create transformers for conversions
        self.world_wgs84_to_enu = Transformer.from_crs("EPSG:4326", self.world_enu_proj.crs, always_xy=True)
        self.world_enu_to_wgs84 = Transformer.from_crs(self.world_enu_proj.crs, "EPSG:4326", always_xy=True)

    def publish_world_origin(self):
        """Publish the world origin as a NavSatFix message"""
        if self.world_origin_lat is None or self.world_origin_lon is None:
            return
            
        world_origin_msg = NavSatFix()
        world_origin_msg.header.stamp = rospy.Time.now()
        world_origin_msg.header.frame_id = self.param_world_link
        world_origin_msg.latitude = self.world_origin_lat
        world_origin_msg.longitude = self.world_origin_lon
        world_origin_msg.altitude = self.world_origin_alt
        world_origin_msg.status.status = 0  # STATUS_FIX
        world_origin_msg.status.service = 1  # SERVICE_GPS
        self.world_origin_pub.publish(world_origin_msg)

    def world_lla_to_enu(self, lat, lon):
        """Convert LLA to world ENU coordinates (altitude ignored)"""
        if self.world_wgs84_to_enu is None:
            return None, None
        
        try:
            east, north = self.world_wgs84_to_enu.transform(lon, lat)
            return east, north
        except Exception as e:
            rospy.logwarn(f"LLA to world ENU conversion failed: {e}")
            return None, None

    def world_enu_to_lla(self, east, north):
        """Convert world ENU coordinates to LLA (returns altitude as world_origin_alt)"""
        if self.world_enu_to_wgs84 is None:
            return None, None, None
        
        try:
            lon, lat = self.world_enu_to_wgs84.transform(east, north)
            return lat, lon, self.world_origin_alt
        except Exception as e:
            rospy.logwarn(f"World ENU to LLA conversion failed: {e}")
            return None, None, None

    def handle_lla_to_enu(self, req):
        """Service handler - converts to local coordinates"""
        try:
            if self.world_enu_proj is None or self.world_to_local_x is None:
                return LLAToUTMResponse(0.0, 0.0, False)

            # Convert to world ENU first
            world_east, world_north = self.world_lla_to_enu(req.latitude, req.longitude)
            
            if world_east is None:
                return LLAToUTMResponse(0.0, 0.0, False)

            # Convert to local coordinates
            local_east = world_east - self.world_to_local_x
            local_north = world_north - self.world_to_local_y

            return LLAToUTMResponse(local_east, local_north, True)

        except Exception as e:
            rospy.logwarn(f"LLA to ENU service conversion failed: {e}")
            return LLAToUTMResponse(0.0, 0.0, False)

    def handle_enu_to_lla(self, req):
        """Service handler - converts from local coordinates"""
        try:
            if self.world_enu_proj is None or self.world_to_local_x is None:
                return UTMToLLAResponse(0.0, 0.0, 0.0, False)

            # Convert from local to world coordinates
            world_east = req.x + self.world_to_local_x
            world_north = req.y + self.world_to_local_y

            # Convert to LLA
            lat, lon, alt = self.world_enu_to_lla(world_east, world_north)
            
            if lat is None:
                return UTMToLLAResponse(0.0, 0.0, 0.0, False)

            return UTMToLLAResponse(lat, lon, True)

        except Exception as e:
            rospy.logwarn(f"ENU to LLA service conversion failed: {e}")
            return UTMToLLAResponse(0.0, 0.0, 0.0, False)

    def imu_callback(self, msg):
        if not self.param_ignore_imu_cov and msg.orientation_covariance[8] > 100:
            return

        self.orientation = msg.orientation

    def gps_callback(self, msg):
        # because mavros is dumb
        if self.param_ignore_status:
            msg.status.status = 0 

        if msg.status.status == -1:
            return
        
        if msg.latitude == 0 and msg.longitude == 0:
            # probably still invalid
            return
        
        self.gps_data = msg
        self.process_data()

    def process_data(self):
        if self.gps_data is None:
            return
        
        # If no world origin is set, determine it automatically or use GPS fix
        if self.world_enu_proj is None:
            if self.param_auto_world_origin:
                # Round to nearest grid point for consistent world origins
                grid_lat, grid_lon = self.round_to_grid(
                    self.gps_data.latitude, 
                    self.gps_data.longitude, 
                    self.param_world_origin_precision
                )
                rospy.loginfo(f"Auto world origin: GPS fix at ({self.gps_data.latitude:.6f}, {self.gps_data.longitude:.6f}) "
                            f"-> grid origin at ({grid_lat:.6f}, {grid_lon:.6f})")
                
                self.world_origin_lat = grid_lat
                self.world_origin_lon = grid_lon
                self.world_origin_alt = self.param_world_altitude
            else:
                rospy.loginfo("No world origin set, using first GPS fix as world origin")
                self.world_origin_lat = self.gps_data.latitude
                self.world_origin_lon = self.gps_data.longitude
                self.world_origin_alt = self.param_world_altitude
            
            self.setup_world_enu_projection()
            self.publish_world_origin()

        # Set local origin on first valid GPS fix
        if self.local_origin_lat is None:
            self.local_origin_lat = self.gps_data.latitude
            self.local_origin_lon = self.gps_data.longitude
            self.local_origin_alt = self.param_world_altitude  # Use consistent altitude
            
            # Calculate world->local transform
            world_east, world_north = self.world_lla_to_enu(
                self.local_origin_lat, 
                self.local_origin_lon
            )
            
            if world_east is not None:
                self.world_to_local_x = world_east
                self.world_to_local_y = world_north
                
                rospy.loginfo(f"Local origin set at: {self.local_origin_lat:.8f}, {self.local_origin_lon:.8f}")
                rospy.loginfo(f"World->Local transform: ({self.world_to_local_x:.2f}, {self.world_to_local_y:.2f})")
                
                # Publish local origin
                self.local_origin_fix = NavSatFix()
                self.local_origin_fix.header = self.gps_data.header
                self.local_origin_fix.header.frame_id = self.param_local_link
                self.local_origin_fix.latitude = self.local_origin_lat
                self.local_origin_fix.longitude = self.local_origin_lon
                self.local_origin_fix.altitude = self.local_origin_alt
                self.local_origin_fix.status = self.gps_data.status
                self.gnss_origin_pub.publish(self.local_origin_fix)

                # Publish static world->local transform
                world_to_local_msg = TransformStamped()
                world_to_local_msg.header.stamp = rospy.Time.now()
                world_to_local_msg.header.frame_id = self.param_world_link
                world_to_local_msg.child_frame_id = self.param_local_link
                world_to_local_msg.transform.translation.x = self.world_to_local_x
                world_to_local_msg.transform.translation.y = self.world_to_local_y
                world_to_local_msg.transform.translation.z = 0.0  # Always zero
                world_to_local_msg.transform.rotation.w = 1.0
                self.static_tf_broadcaster.sendTransform(world_to_local_msg)

        # Convert current position to world ENU, then to local coordinates
        if self.world_to_local_x is None:
            return  # Local origin not established yet

        world_east, world_north = self.world_lla_to_enu(
            self.gps_data.latitude, 
            self.gps_data.longitude
        )
        
        if world_east is None:
            rospy.logwarn("Failed to convert GPS position to world ENU")
            return

        # Convert to local coordinates
        local_east = world_east - self.world_to_local_x
        local_north = world_north - self.world_to_local_y

        cov = [0.0] * 36
        cov[0] = math.sqrt(math.fabs(self.gps_data.position_covariance[0]))
        cov[7] = math.sqrt(math.fabs(self.gps_data.position_covariance[4]))
        cov[14] = math.sqrt(math.fabs(self.gps_data.position_covariance[8]))

        if self.param_publish_tf:
            local_to_base_msg = TransformStamped()
            local_to_base_msg.header.stamp = rospy.Time.now()
            local_to_base_msg.header.frame_id = self.param_local_link
            local_to_base_msg.child_frame_id = self.param_base_link
            local_to_base_msg.transform.translation.x = local_east
            local_to_base_msg.transform.translation.y = local_north
            local_to_base_msg.transform.translation.z = 0.0  # Always zero
            local_to_base_msg.transform.rotation = self.orientation
            self.tf_broadcaster.sendTransform(local_to_base_msg)

        if self.param_publish_odom:
            odom_msg = Odometry()
            odom_msg.header.frame_id = self.param_local_link
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.pose.pose.position.x = local_east
            odom_msg.pose.pose.position.y = local_north
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
        pose_msg.pose.pose.position.x = local_east
        pose_msg.pose.pose.position.y = local_north
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation = self.orientation
        pose_msg.pose.covariance = cov
        self.pose_pub.publish(pose_msg)

node = GNSSENUNode()
rospy.spin()