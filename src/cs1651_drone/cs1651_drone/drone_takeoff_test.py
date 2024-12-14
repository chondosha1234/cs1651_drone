import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandTOLLocal
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import NavSatFix
from time import sleep

armed = False

class DroneNode(Node):

    def __init__(self):
        super().__init__('drone_node')
        
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set mode service.")
        
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for the arming service.")
        
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for takeoff service.")

        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for land service.")


        qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
        )

        
        self.position_pub = self.create_publisher(
                PoseStamped,
                '/mavros/setpoint_position/local',
                10
        )

        self.state_sub = self.create_subscription(
                State,
                '/mavros/state',
                self.state_callback,
                qos_profile
        )
        self.state_sub

        self.local_pose_sub = self.create_subscription(
                PoseStamped,
                '/mavros/local_position/pose',
                self.pose_callback,
                qos_profile
        )
        self.local_pose_sub
        
        self.gps_sub = self.create_subscription(
                NavSatFix,
                '/mavros/global_position/global',
                self.gps_fix_callback,
                qos_profile
        )
        self.gps_sub

        self.altitude = 0.0
        self.start_altitude = 0.0
        self.target_altitude = 0.0
        self.lat = 0.0
        self.long = 0.0

        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.0

    def state_callback(self, msg):
        #armed = msg.armed
        return


    def pose_callback(self, msg):
        #self.get_logger().info(f"altitude: {msg.pose.position.z}")
        self.curr_x = msg.pose.position.x
        self.curr_y = msg.pose.position.y
        self.curr_z = msg.pose.position.z
        return

    def gps_fix_callback(self, msg):
        #self.get_logger().info(f"GPS altitude: {msg.altitude}")
        if self.start_altitude == 0.0:
            self.start_altitude = msg.altitude
        self.altitude = msg.altitude

        if self.lat == 0.0:
            self.lat = msg.latitude

        if self.long == 0.0:
            self.long = msg.longitude
        

    def set_mode(self, mode):
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"Mode set to {mode} successfully.")
        else:
            self.get_logger().info(f"Failed to set mode to {mode}.")
    

    # called after spinning a few times after takeoff to try to set an accurate x,y,z origin to use as reference
    def set_origin_point(self):
        self.origin_x = self.curr_x
        self.origin_y = self.curr_y
        self.origin_z = self.curr_z
        self.get_logger().info(f"Origin set at: x: {self.origin_x}  y: {self.origin_y}   z: {self.origin_z}")


    def arm_drone(self, armed):
        request = CommandBool.Request()
        request.value = armed
        
        while True:
            
            if armed:
                self.get_logger().info("Attempting to arm drone.")
            else:
                self.get_logger().info("Attempting to disarm drone.")

            future = self.arm_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() and future.result().success:
                if armed:
                    self.get_logger().info("Drone armed successfully!")
                else:
                    self.get_logger().info("Drone disarmed successfully!")
                break
            elif future.result() and not future.result().success:
                self.get_logger().info("Failed to change arming state. Future failed")
            else:
                self.get_logger().info("Failed to change arming state. Future didn't return")
           
            rclpy.spin_once(self)
            sleep(2)

    
    def takeoff2(self):
        
        takeoff_pos = PoseStamped()
        takeoff_pos.pose.position.x = 0.0
        takeoff_pos.pose.position.y = 0.0
        takeoff_pos.pose.position.z = 1.0

        for i in range(10):
            self.position_pub.publish(takeoff_pos)
            rclpy.spin_once(self)
            sleep(.10)

        self.set_mode("OFFBOARD")

        self.arm_drone(True)

        while self.altitude < self.target_altitude:
            self.set_mode("OFFBOARD")
            self.position_pub.publish(takeoff_pos)
            rclpy.spin_once(self)
            sleep(.10)


    def takeoff(self, height):
        #while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info("Waiting for takeoff service.")
        
        self.target_altitude = self.altitude + height
        self.get_logger().info(f"set altitude: {self.altitude+height}")
        #self.target_altitude = self.altitude + 1.0

        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = self.lat
        request.longitude = self.long
        request.altitude = self.target_altitude

        self.get_logger().info("Starting takeoff...")
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("Takeoff command received.")
        elif future.result() and not future.result().success:
            self.get_logger().info(f"Takeoff command failed. Future returned: {future.result()}")
        else:
            self.get_logger().info(f"Takeoff command failed. Future didn't return: {future.result()}")


    def land(self):
    
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = self.lat
        request.longitude = self.long
        request.altitude = self.start_altitude + 0.2

        self.get_logger().info("Attempting to land...")
        future = self.land_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info(f"Land command received. Future returned: {future.result()}")
        else:
            self.get_logger().info("Land command failed.")
    

    def move_to_pos(self, x, y, z):
        target_pos = PoseStamped()
        target_pos.pose.position.x = self.origin_x + x
        target_pos.pose.position.y = self.origin_y + y
        target_pos.pose.position.z = self.origin_z + z
        
        self.get_logger().info("starting 1 sec position prep")
        for i in range(5):
            self.position_pub.publish(target_pos)
            rclpy.spin_once(self)
            sleep(.10)

        self.set_mode("OFFBOARD")
        
        self.get_logger().info("Starting 2 seconds of moving.")
        for i in range(20):
            self.position_pub.publish(target_pos)
            rclpy.spin_once(self)
            sleep(.10)

        self.set_mode("AUTO.LOITER")


def main(args=None):
    
    global target_altitude

    rclpy.init(args=args)
    node = DroneNode()
    
    # spin to set some initial parameters
    rclpy.spin_once(node)
    node.set_mode("MANUAL")
    
    # arm drone
    node.arm_drone(True)
    
    # spin a couple of times to set altitude parameters
    #rclpy.spin_once(node)
    #rclpy.spin_once(node)
    while node.altitude < 50.0:
        rclpy.spin_once(node)

    # take off to given height
    node.takeoff(2.0)
     
    while node.altitude < node.target_altitude:
        node.get_logger().info(f"target: {node.target_altitude}   alt: {node.altitude}")
        rclpy.spin_once(node)
        sleep(0.1) # need 10hz freq 
    
    # spin a few times after takeoff to set origin
    rclpy.spin_once(node)
    rclpy.spin_once(node)
    rclpy.spin_once(node)
    
    # set whatever origin point to use as reference in moves
    node.set_origin_point()
    
    # try to move 1 meter in 1 direction
    node.move_to_pos(0.0, 4.0, 0.0)
    
    rclpy.spin_once(node)
    
    #3 return to origin
    node.move_to_pos(0.0, 0.0, 0.0)

    node.land()

    while node.altitude > node.start_altitude + 0.2:
        node.get_logger().info(f"target: {node.start_altitude + 0.2}   alt: {node.altitude}")
        rclpy.spin_once(node)
        sleep(0.1)
    
    node.arm_drone(False)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



