import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, LineString, Point
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import threading

# Boundary points from the document (latitude, longitude)
boundary_points = [
    (-35.3631093, 149.1648396),
    (-35.3628754, 149.1648359),
    (-35.3626198, 149.1648281),
    (-35.3625246, 149.1647141),
    (-35.3624976, 149.1644084),
    (-35.3626432, 149.1644081),
    (-35.3627150, 149.1644138),
    (-35.3628831, 149.1644189),
]

class DroneLawnmower(Node):
    def __init__(self):
        super().__init__('drone_lawnmower')
        print("Initialization done")

        # DroneKit vehicle connection
        self.connection_string = "127.0.0.1:14550"
        self.vehicle = None
        self.altitude = 10.0  # Fixed altitude in meters
        self.sweep_spacing = 0.2

        # Consistent reference point (AirSim OriginGeopoint)
        self.ref_lat = -35.363261
        self.ref_lon = 149.165230

        # Point cloud storage
        self.point_cloud = []  # List to store (lat, lon, z) points
        self.drone_position = None  # Store drone's (lat, lon) for alignment
        self.collect_pointcloud = False  # Start with collection disabled
        self.subscriber = self.create_subscription(
            PointCloud2,
            '/airsim/pointcloud',
            self.point_cloud_callback,
            10
        )
        print("Subscription Created")

        # Publisher for RViz
        self.publisher_ = self.create_publisher(PointCloud2, '/drone_pointcloud', 10)

        # Generate lawnmower path with consistent reference
        self.waypoints = self.generate_lawnmower_path(boundary_points, self.sweep_spacing)
        print("WayPoints Generated")

        # Connect to vehicle early to get initial position
        self.connect_vehicle()
        if self.vehicle:
            self.drone_position = (
                self.vehicle.location.global_relative_frame.lat,
                self.vehicle.location.global_relative_frame.lon
            )
            print(f"Initial drone position: {self.drone_position}")

        # Start a background thread for ROS 2 spinning
        self.spinning = True
        self.spin_thread = threading.Thread(target=self.spin_node)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        self.get_logger().info('Drone lawnmower node initialized.')

    def spin_node(self):
        """Run ROS 2 node in background to collect point cloud data."""
        while self.spinning and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def shutdown(self):
        """Stop spinning and shutdown node."""
        self.spinning = False
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=2.0)
        if self.vehicle:
            self.vehicle.close()
        self.destroy_node()

    def connect_vehicle(self):
        """Connect to the vehicle via DroneKit."""
        print("Attempting to connect to vehicle...")
        for _ in range(5):
            try:
                self.vehicle = connect(self.connection_string, wait_ready=True)
                print("Configuring vehicle parameters...")
                self.vehicle.parameters['ARMING_CHECK'] = 0
                print("Vehicle connected!")
                return self.vehicle
            except Exception as e:
                print(f"Connection failed: {e}. Retrying in 2 seconds...")
                time.sleep(2)
        raise Exception("Failed to connect after retries.")

    def point_cloud_callback(self, msg):
        """Callback to process PointCloud2 messages and convert to lat/lon with drone position."""
        if not self.collect_pointcloud:
            return
        if not self.drone_position or not self.vehicle:
            self.get_logger().warn("No drone position available for point cloud alignment")
            return

        drone_lat, drone_lon = self.drone_position
        point_step = msg.point_step
        data = msg.data
        num_points = len(data) // point_step
        sample_rate = 80  # Collect every 10th point to reduce data volume

        for i in range(0, num_points, sample_rate):  # Downsample points
            offset = i * point_step
            x = struct.unpack('f', data[offset:offset+4])[0]  # x in meters
            y = struct.unpack('f', data[offset+4:offset+8])[0]  # y in meters
            z = struct.unpack('f', data[offset+8:offset+12])[0]  # z in meters
            delta_lat, delta_lon = self.meters_to_latlon(x, y, 0.0, 0.0)
            lat = drone_lat + delta_lat
            lon = drone_lon + delta_lon
            self.point_cloud.append((lat, lon, z))
            self.publish_pointcloud(lat, lon, z)
        self.get_logger().info(f"Point cloud size: {len(self.point_cloud)} points")

    def publish_pointcloud(self, lat, lon, z):
        """Publish point cloud data for RViz."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        x, y = self.latlon_to_meters(lat, lon, self.ref_lat, self.ref_lon)
        data = struct.pack('fff', float(x), float(y), float(z))
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = 1
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 12
        pc2.row_step = pc2.point_step
        pc2.data = data
        pc2.is_dense = True
        self.publisher_.publish(pc2)

    def latlon_to_meters(self, lat, lon, ref_lat, ref_lon):
        """Convert latitude/longitude to meters relative to a reference point."""
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)
        ref_lat_rad = np.radians(ref_lat)
        ref_lon_rad = np.radians(ref_lon)
        R = 6378137.0
        x = R * (lon_rad - ref_lon_rad) * np.cos(ref_lat_rad)
        y = R * (lat_rad - ref_lat_rad)
        return x, y

    def meters_to_latlon(self, x, y, ref_lat, ref_lon):
        """Convert meters back to latitude/longitude."""
        R = 6378137.0
        ref_lat_rad = np.radians(ref_lat)
        lat = ref_lat + (y / R) * (180 / np.pi)
        lon = ref_lon + (x / (R * np.cos(ref_lat_rad))) * (180 / np.pi)
        return lat - ref_lat, lon - ref_lon

    def generate_lawnmower_path(self, boundary, sweep_spacing):
        """Generate lawnmower waypoints within the boundary using consistent reference."""
        boundary_meters = [self.latlon_to_meters(lat, lon, self.ref_lat, self.ref_lon) for lat, lon in boundary]
        poly = Polygon(boundary_meters)
        print("polygon generated")
        min_x, min_y, max_x, max_y = poly.bounds
        print("Min_x =", min_x)
        print("Max_x =", max_x)
        waypoints_meters = []
        x = min_x
        direction = 1
        while x <= max_x:
            line = LineString([(x, min_y), (x, max_y)])
            clipped = poly.intersection(line)
            if clipped.is_empty:
                x += sweep_spacing
                continue
            if clipped.geom_type == 'LineString':
                y1, y2 = clipped.xy[1]
                waypoints_meters.append((x, y1))
                waypoints_meters.append((x, y2))
            elif clipped.geom_type == 'MultiLineString':
                for l in clipped.geoms:
                    y1, y2 = l.xy[1]
                    waypoints_meters.append((x, y1))
                    waypoints_meters.append((x, y2))
            x += sweep_spacing
            direction *= -1
            waypoints_meters = sorted(waypoints_meters, key=lambda p: p[1] * direction)
        print("WayPoints Generated")
        waypoints = [self.meters_to_latlon(x, y, self.ref_lat, self.ref_lon) for x, y in waypoints_meters]
        return [(self.ref_lat + lat_delta, self.ref_lon + lon_delta) for lat_delta, lon_delta in waypoints]

    def arm_and_takeoff(self, target_altitude):
        """Arm the vehicle and take off."""
        print("Setting GUIDED mode...")
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode.name != "GUIDED":
            print("Waiting for GUIDED mode...")
            time.sleep(1)
        print("Arming motors...")
        self.vehicle.parameters['ARMING_CHECK'] = 0
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)
        print("Taking off...")
        self.vehicle.simple_takeoff(target_altitude)
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print(f"Altitude: {alt}")
            if alt >= target_altitude * 0.95:
                print("Reached target altitude")
                self.collect_pointcloud = True
                break
            time.sleep(1)

    def send_waypoints(self):
        """Send waypoints to the vehicle."""
        print("Sending waypoints...")
        cmds = self.vehicle.commands
        cmds.clear()
        for lat, lon in self.waypoints:
            point = LocationGlobalRelative(lat, lon, self.altitude)
            cmds.add(dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                                     lat, lon, self.altitude))
        cmds.upload()
        self.get_logger().info("Waypoints uploaded")

    def execute_mission(self):
        """Execute the mission."""
        print("Starting mission...")
        self.vehicle.commands.next = 0
        self.vehicle.mode = VehicleMode("AUTO")
        self.collect_pointcloud = True
        while True:
            next_cmd = self.vehicle.commands.next
            self.get_logger().info(f"Executing waypoint {next_cmd}")
            if next_cmd >= len(self.vehicle.commands):
                self.get_logger().info("Mission complete")
                self.collect_pointcloud = False
                break
            self.drone_position = (
                self.vehicle.location.global_relative_frame.lat,
                self.vehicle.location.global_relative_frame.lon
            )
            time.sleep(1)
        print("Returning to launch...")
        self.vehicle.mode = VehicleMode("RTL")
        while self.vehicle.location.global_relative_frame.alt > 0.5:
            print(f"Altitude: {self.vehicle.location.global_relative_frame.alt}")
            self.drone_position = (
                self.vehicle.location.global_relative_frame.lat,
                self.vehicle.location.global_relative_frame.lon
            )
            time.sleep(1)
        print("Landed.")

    def plot_all(self):
        """Plot boundary/path, 3D Depth Model (removing large depth outliers), and 2D heatmap DEM as subplots."""
        print("Entering plot_all...")
        fig = plt.figure(figsize=(15, 5), constrained_layout=True)
    
        # Subplot 1: Boundary and Path (2D)
        ax1 = fig.add_subplot(131)
        boundary_lats, boundary_lons = zip(*boundary_points)
        # Convert boundary to meters for consistent scaling
        boundary_meters = [self.latlon_to_meters(lat, lon, self.ref_lat, self.ref_lon) for lat, lon in zip(boundary_lats, boundary_lons)]
        bx, by = zip(*boundary_meters)
        ax1.plot(bx, by, 'b-', label='Boundary')
        ax1.plot(bx + (bx[0],), by + (by[0],), 'b-')
        # Convert waypoints to meters
        wp_meters = [self.latlon_to_meters(lat, lon, self.ref_lat, self.ref_lon) for lat, lon in self.waypoints]
        wx, wy = zip(*wp_meters)
        ax1.plot(wx, wy, 'g*-', label='Waypoints')
        if self.point_cloud:
            pc_lats, pc_lons = zip(*[(p[0], p[1]) for p in self.point_cloud])
            # Convert point cloud to meters
            pc_meters = [self.latlon_to_meters(lat, lon, self.ref_lat, self.ref_lon) for lat, lon in zip(pc_lats, pc_lons)]
            px, py = zip(*pc_meters)
            ax1.scatter(px, py, c='r', marker='.', label='Point Cloud', s=10)
            print(f"Point cloud plotted with {len(self.point_cloud)} points.")
        else:
            print("No point cloud data to plot.")
        ax1.set_xlabel('X (meters)')
        ax1.set_ylabel('Y (meters)')
        ax1.set_title('Lawnmower Path with Boundary')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')
        # Set limits based on combined extent
        all_x = bx + wx + px if self.point_cloud else bx + wx
        all_y = by + wy + py if self.point_cloud else by + wy
        x_min, x_max = min(all_x), max(all_x)
        y_min, y_max = min(all_y), max(all_y)
        ax1.set_xlim(x_min - 10, x_max + 10)
        ax1.set_ylim(y_min - 10, y_max + 10)
    
        # Subplot 2: 3D Depth Model (removing large depth outliers)
        if self.point_cloud:
            ax2 = fig.add_subplot(132, projection='3d')
            pc_lats, pc_lons, pc_zs = zip(*self.point_cloud)
            x, y = self.latlon_to_meters(np.array(pc_lats), np.array(pc_lons), self.ref_lat, self.ref_lon)
            z = np.array(pc_zs)
            # Negate z to represent depth (positive downward)
            z_depth = -z
        
            # Remove large depth outliers (e.g., >10m)
            depth_threshold = -10.0  # Max realistic depth in meters
            non_outlier_mask = z_depth >= depth_threshold
            num_outliers = len(z_depth) - np.sum(non_outlier_mask)
            print(f"Removed {num_outliers} large depth outliers (> {depth_threshold}m).")
            # Replace outliers with median of non-outlier depths
            median_depth = np.median(z_depth[non_outlier_mask])
            z_depth_clean = np.where(non_outlier_mask, z_depth, median_depth)
        
            # Plot cleaned depth values
            scatter = ax2.scatter(x, y, z_depth_clean, c=z_depth_clean, cmap='viridis', marker='.', s=10)
            ax2.set_xlabel('X (meters)')
            ax2.set_ylabel('Y (meters)')
            ax2.set_zlabel('Depth (m)')
            ax2.set_title('3D Depth Model (Large Outliers Removed)')
            fig.colorbar(scatter, ax=ax2, label='Depth (m)')
        else:
            print("No point cloud data for 3D Depth Model.")
    
        # Subplot 3: 2D Heatmap DEM (Depth with large outliers removed)
        if self.point_cloud:
            ax3 = fig.add_subplot(133)
            x, y = self.latlon_to_meters(np.array(pc_lats), np.array(pc_lons), self.ref_lat, self.ref_lon)
            z = np.array(pc_zs)
            z_depth = -z  # Negate for depth
            # Use same threshold as 3D plot
            depth_threshold = -10.0  # Max realistic depth in meters
            non_outlier_mask = z_depth >= depth_threshold
            z_depth_clean = np.where(non_outlier_mask, z_depth, median_depth)
            scatter = ax3.scatter(x, y, c=z_depth_clean, cmap='viridis', marker='.', s=10)
            ax3.set_xlabel('X (meters)')
            ax3.set_ylabel('Y (meters)')
            ax3.set_title('2D Heatmap Depth Model (Large Outliers Removed)')
            fig.colorbar(scatter, ax=ax3, label='Depth (m)')
            ax3.axis('equal')
            ax3.grid(True)
        else:
            print("No point cloud data for 2D Heatmap DEM.")
    
        print("Calling plt.show() for all plots...")
        plt.show()
        print("plt.show() completed.")

    def run(self):
        """Run the drone mission."""
        try:
            self.arm_and_takeoff(self.altitude)
            self.send_waypoints()
            self.execute_mission()
            print("Mission finished, plotting results...")
            self.plot_all()
        finally:
            self.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DroneLawnmower()
    print("Object Created")
    try:
        node.run()
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
        node.shutdown()
    finally:
        rclpy.shutdown()
        print("ROS 2 shutdown completed.")

if __name__ == '__main__':
    main()
