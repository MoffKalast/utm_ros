#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from pyproj import Transformer
import threading

class ProjectionErrorAnalyzer:
    def __init__(self):
        rospy.init_node('projection_error_analyzer', anonymous=True)
        
        # Transformers for coordinate conversions
        self.wgs84_to_ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
        
        # Data storage
        self.gnss_fix_data = []
        self.enu_error_data = []
        self.utm_error_data = []
        self.distances = []  # Distance from origin
        self.timestamps = []
        
        # Local origins
        self.enu_origin = None
        self.utm_origin = None
        self.enu_origin_ecef = None
        self.utm_origin_ecef = None
        
        # UTM zone info (will be determined from first fix)
        self.utm_zone = None
        self.utm_transformer = None
        
        # Locks for thread safety
        self.data_lock = threading.Lock()
        
        # Current messages
        self.current_gnss_fix = None
        self.current_enu_odom = None
        self.current_utm_odom = None
        
        # Subscribers
        rospy.Subscriber('/gnss/fix', NavSatFix, self.gnss_fix_callback)
        rospy.Subscriber('/gnss/odom', Odometry, self.enu_odom_callback)
        rospy.Subscriber('/gnss_utm/odom', Odometry, self.utm_odom_callback)
        rospy.Subscriber('/gnss/local_origin_fix', NavSatFix, self.enu_origin_callback)
        rospy.Subscriber('/gnss_utm/local_origin_fix', NavSatFix, self.utm_origin_callback)
        
        rospy.loginfo("Projection Error Analyzer initialized. Waiting for local origins...")
        
    def enu_origin_callback(self, msg):
        """Store ENU local origin and convert to ECEF"""
        if self.enu_origin is None:
            self.enu_origin = msg
            x, y, z = self.wgs84_to_ecef.transform(msg.longitude, msg.latitude, msg.altitude)
            self.enu_origin_ecef = np.array([x, y, z])
            rospy.loginfo(f"ENU origin set: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")
    
    def utm_origin_callback(self, msg):
        """Store UTM local origin and setup UTM transformer"""
        if self.utm_origin is None:
            self.utm_origin = msg
            x, y, z = self.wgs84_to_ecef.transform(msg.longitude, msg.latitude, msg.altitude)
            self.utm_origin_ecef = np.array([x, y, z])
            
            # Determine UTM zone from origin
            import utm as utm_lib
            _, _, zone_number, zone_letter = utm_lib.from_latlon(msg.latitude, msg.longitude)
            self.utm_zone = f"{zone_number}{zone_letter}"
            
            # Create proper UTM to ECEF transformer
            # UTM zones are EPSG:326XX for north, 327XX for south (XX = zone number)
            hemisphere = 6 if ord(zone_letter) >= ord('N') else 7
            epsg_code = f"EPSG:32{hemisphere}{zone_number:02d}"
            self.utm_transformer = Transformer.from_crs(epsg_code, "EPSG:4978", always_xy=True)
            
            rospy.loginfo(f"UTM origin set: lat={msg.latitude}, lon={msg.longitude}, zone={self.utm_zone}, EPSG={epsg_code}")
    
    def gnss_fix_callback(self, msg):
        """Store current GNSS fix"""
        self.current_gnss_fix = msg
        self.process_data()
    
    def enu_odom_callback(self, msg):
        """Store current ENU odometry"""
        self.current_enu_odom = msg
    
    def utm_odom_callback(self, msg):
        """Store current UTM odometry"""
        self.current_utm_odom = msg
    
    def navsat_to_ecef(self, navsat_msg):
        """Convert NavSatFix message to ECEF coordinates"""
        x, y, z = self.wgs84_to_ecef.transform(
            navsat_msg.longitude, 
            navsat_msg.latitude, 
            navsat_msg.altitude
        )
        return np.array([x, y, z])
    
    def enu_to_ecef(self, enu_odom, origin_ecef, origin_navsat):
        """Convert ENU odometry to ECEF coordinates"""
        if origin_ecef is None or origin_navsat is None:
            return None
        
        # Extract ENU position from odometry
        enu_x = enu_odom.pose.pose.position.x
        enu_y = enu_odom.pose.pose.position.y
        enu_z = enu_odom.pose.pose.position.z
        enu_pos = np.array([enu_x, enu_y, enu_z])
        
        # Calculate rotation matrix from ENU to ECEF
        lat = np.radians(origin_navsat.latitude)
        lon = np.radians(origin_navsat.longitude)
        
        # Rotation matrix from ENU to ECEF
        R = np.array([
            [-np.sin(lon), -np.sin(lat)*np.cos(lon), np.cos(lat)*np.cos(lon)],
            [np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)*np.sin(lon)],
            [0, np.cos(lat), np.sin(lat)]
        ])
        
        # Transform ENU to ECEF
        ecef_offset = R @ enu_pos
        ecef_pos = origin_ecef + ecef_offset
        
        return ecef_pos
    
    def utm_to_ecef(self, utm_odom, origin_ecef, origin_navsat):
        """Convert UTM odometry to ECEF coordinates using proper UTM projection"""
        if origin_ecef is None or origin_navsat is None or self.utm_transformer is None:
            return None
        
        # Get local UTM coordinates from odometry
        local_x = utm_odom.pose.pose.position.x
        local_y = utm_odom.pose.pose.position.y
        local_z = utm_odom.pose.pose.position.z
        
        # Convert origin to UTM coordinates
        import utm as utm_lib
        origin_easting, origin_northing, _, _ = utm_lib.from_latlon(
            origin_navsat.latitude, 
            origin_navsat.longitude
        )
        
        # Add local offsets to origin UTM coordinates
        utm_easting = origin_easting + local_x
        utm_northing = origin_northing + local_y
        utm_altitude = origin_navsat.altitude + local_z
        
        # Convert UTM to ECEF using pyproj
        x, y, z = self.utm_transformer.transform(utm_easting, utm_northing, utm_altitude)
        
        return np.array([x, y, z])
    
    def process_data(self):
        """Process current messages and calculate errors"""
        if self.current_gnss_fix is None or self.current_enu_odom is None or self.current_utm_odom is None:
            return
        
        if self.enu_origin_ecef is None or self.utm_origin_ecef is None:
            return
        
        # Convert GNSS fix to ECEF (ground truth)
        gnss_ecef = self.navsat_to_ecef(self.current_gnss_fix)
        
        # Convert ENU odometry to ECEF
        enu_ecef = self.enu_to_ecef(self.current_enu_odom, self.enu_origin_ecef, self.enu_origin)
        
        # Convert UTM odometry to ECEF (now with proper UTM handling)
        utm_ecef = self.enu_to_ecef(self.current_utm_odom, self.utm_origin_ecef, self.utm_origin)
        
        if enu_ecef is None or utm_ecef is None:
            return
        
        # Calculate errors (Euclidean distance)
        enu_error = np.linalg.norm(enu_ecef - gnss_ecef)
        utm_error = np.linalg.norm(utm_ecef - gnss_ecef)
        
        # Calculate distance from origin
        distance_from_origin = np.linalg.norm(gnss_ecef - self.enu_origin_ecef)
        
        # Store data
        with self.data_lock:
            self.timestamps.append(rospy.get_time())
            self.enu_error_data.append(enu_error)
            self.utm_error_data.append(utm_error)
            self.distances.append(distance_from_origin)
            self.gnss_fix_data.append(gnss_ecef)
        
        rospy.loginfo_throttle(1.0, f"Distance: {distance_from_origin:.1f}m | ENU Error: {enu_error:.3f}m, UTM Error: {utm_error:.3f}m")
    
    def update_plot(self, frame):
        """Update plot with new data"""
        with self.data_lock:
            if len(self.timestamps) == 0:
                return
            
            distances = np.array(self.distances)
            enu_errors = np.array(self.enu_error_data)
            utm_errors = np.array(self.utm_error_data)
        
        self.ax1.clear()
        self.ax1.plot(distances, enu_errors, 'b.', label='ENU Error', markersize=3, alpha=0.6)
        self.ax1.plot(distances, utm_errors, 'r.', label='UTM Error', markersize=3, alpha=0.6)
        self.ax1.set_xlabel('Distance from Origin (m)')
        self.ax1.set_ylabel('Error (m)')
        self.ax1.set_title('Projection Error vs Distance from Origin')
        self.ax1.legend()
        self.ax1.grid(True)
        
        self.ax2.clear()
        if len(enu_errors) > 0 and len(utm_errors) > 0:
            self.ax2.hist([enu_errors, utm_errors], bins=30, label=['ENU', 'UTM'], alpha=0.7)
            self.ax2.set_xlabel('Error (m)')
            self.ax2.set_ylabel('Frequency')
            self.ax2.set_title('Error Distribution')
            self.ax2.legend()
            self.ax2.grid(True)
            
            # Add statistics
            stats_text = f'ENU: mean={np.mean(enu_errors):.3f}m, std={np.std(enu_errors):.3f}m, max={np.max(enu_errors):.3f}m\n'
            stats_text += f'UTM: mean={np.mean(utm_errors):.3f}m, std={np.std(utm_errors):.3f}m, max={np.max(utm_errors):.3f}m\n'
            stats_text += f'Max distance: {np.max(distances):.1f}m'
            self.ax2.text(0.02, 0.98, stats_text, transform=self.ax2.transAxes, 
                    verticalalignment='top', fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    def run(self):
        """Main run loop with plotting in main thread"""
        # Set up the plot
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        plt.tight_layout()
        
        # Start ROS spinning in a separate thread
        ros_thread = threading.Thread(target=rospy.spin)
        ros_thread.daemon = True
        ros_thread.start()
        
        # Set up animation
        ani = FuncAnimation(self.fig, self.update_plot, interval=1000, cache_frame_data=False)
        
        # Show plot
        plt.show(block=True)

if __name__ == '__main__':
    try:
        analyzer = ProjectionErrorAnalyzer()
        analyzer.run()
    except rospy.ROSInterruptException:
        pass