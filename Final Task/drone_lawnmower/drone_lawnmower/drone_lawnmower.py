import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString, Point
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
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
        self.altitude = 5.0  # Fixed altitude in meters
        self.sweep_spacing = 0.1202

        # Consistent reference point (AirSim OriginGeopoint)
        self.ref_lat = -35.363261
        self.ref_lon = 149.165230

        # Point cloud storage
        self.point_cloud = []  # List to store (lat, lon) points
        self.drone_position = None  # Store drone's (lat, lon) for alignment
        self.collect_pointcloud = False  # Start with collection disabled
        self.subscriber = self.create_subscription(
            PointCloud2,
            '/airsim/pointcloud',
            self.point_cloud_callback,
            10
        )
        print("Subscription Created")

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

        for i in range(num_points):
            offset = i * point_step
            x = struct.unpack('f', data[offset:offset+4])[0]  # x in meters (Lidar local frame)
            y = struct.unpack('f', data[offset+4:offset+8])[0]  # y in meters
            # Convert Lidar local (x, y) to global (lat, lon) relative to drone position
            delta_lat, delta_lon = self.meters_to_latlon(x, y, 0.0, 0.0)
            lat = drone_lat + delta_lat
            lon = drone_lon + delta_lon
            self.point_cloud.append((lat, lon))
        self.get_logger().info(f"Point cloud size: {len(self.point_cloud)} points")

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
        return lat - ref_lat, lon - ref_lon  # Return delta for relative offset

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
                self.collect_pointcloud = True  # Enable after takeoff
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
        self.collect_pointcloud = True  # Enable collection at mission start
        while True:
            next_cmd = self.vehicle.commands.next
            self.get_logger().info(f"Executing waypoint {next_cmd}")
            if next_cmd >= len(self.vehicle.commands):
                self.get_logger().info("Mission complete")
                self.collect_pointcloud = False  # Disable after mission complete
                break
            # Update drone position during mission
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

    def plot_boundary_and_path(self):
        """Plot the boundary, waypoints, and point cloud (matplotlib)."""
        print("Entering plot_boundary_and_path...")
        plt.figure(figsize=(10, 10))
        boundary_lats, boundary_lons = zip(*boundary_points)
        plt.plot(boundary_lons, boundary_lats, 'b-', label='Boundary')
        plt.plot(boundary_lons + (boundary_lons[0],), boundary_lats + (boundary_lats[0],), 'b-')
        wp_lats, wp_lons = zip(*self.waypoints)
        plt.plot(wp_lons, wp_lats, 'g*-', label='Waypoints')
        if self.point_cloud:
            pc_lats, pc_lons = zip(*self.point_cloud)
            plt.scatter(pc_lons, pc_lats, c='r', marker='.', label='Point Cloud', s=10)
            print(f"Point cloud plotted with {len(self.point_cloud)} points.")
        else:
            print("No point cloud data to plot.")
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title('Lawnmower Path with Boundary and Point Cloud')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        print("Calling plt.show()...")
        plt.show()
        print("plt.show() completed.")

    def run(self):
        """Run the drone mission."""
        try:
            # Arm and takeoff
            self.arm_and_takeoff(self.altitude)
            # Send waypoints
            self.send_waypoints()
            # Execute mission
            self.execute_mission()
            # Plot after mission
            print("Mission finished, plotting results...")
            self.plot_boundary_and_path()
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
