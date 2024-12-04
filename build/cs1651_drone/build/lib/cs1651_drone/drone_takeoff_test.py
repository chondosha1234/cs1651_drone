import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.srv import CommandTOLLocal
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import NavSatFix
from time import sleep

armed = False

target_altitude = 1.0
tolerance = 0.2

current_altitude = 0.0

#status = -1


class DroneNode(Node):

    def __init__(self):
        super().__init__('drone_node')
        
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set mode service.")
        
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for the arming service.")
        
        self.takeoff_client = self.create_client(CommandTOLLocal, '/mavros/cmd/takeoff_local')
#        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
 #           self.get_logger().info("Waiting for takeoff service.")

        self.land_client = self.create_client(CommandTOLLocal, '/mavros/cmd/land_local')
  #      while not self.land_client.wait_for_service(timeout_sec=1.0):
   #         self.get_logger().info("Waiting for land service.")


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

        self.altitude_sub = self.create_subscription(
                PoseStamped,
                '/mavros/local_position/pose',
                self.alt_callback,
                qos_profile
        )
        self.altitude_sub
        
        self.gps_sub = self.create_subscription(
                NavSatFix,
                '/mavros/global_position/raw/fix',
                self.gps_fix_callback,
                qos_profile
        )
        self.gps_sub

        self.status = -1

    def state_callback(self, msg):
        #armed = msg.armed
        return


    def alt_callback(self, msg):
        self.get_logger().info(f"altitude: {msg.pose.position.z}")
        current_altitude = msg.pose.position.z

    def gps_fix_callback(self, msg):
        self.get_logger().info(f"GPS fix status: {msg.status.status}")
        self.status = msg.status.status

    def set_mode(self, mode):
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"Mode set to {mode} successfully.")
        else:
            self.get_logger().info(f"Failed to set mode to {mode}.")


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
            else:
                self.get_logger().info("Failed to change arming state")
            
            sleep(2)


    def takeoff(self, target_height):
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for takeoff service.")

        request = CommandTOLLocal.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.rate = 0.2
        request.position.x = 0.0
        request.position.y = 0.0
        request.position.z = 1.0

        self.get_logger().info("Starting takeoff...")
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("Takeoff command received.")
        else:
            self.get_logger().info("Takeoff command failed.")


    def land(self):
        request = CommandTOLLocal.Request()
        request.offset = 0.1
        request.yaw = 0.0
        request.rate = 0.2
        request.position.x = 0.0
        request.position.y = 0.0
        request.position.z = 0.0

        self.get_logger().info("Attempting to land...")
        future = self.land_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("Land command received.")
        else:
            self.get_logger().info("Land command failed.")


def main(args=None):
    
    global target_altitude

    rclpy.init(args=args)
    node = DroneNode()
    
    #while node.status < 1:
     #   node.get_logger().info(f"Waiting for GPS fix...  status: {node.status}")
      #  rclpy.spin_once(node, timeout_sec=1.0)

    node.arm_drone(True)
#    node.set_mode("OFFBOARD")

    #while current_altitude < (target_altitude - tolerance):
    node.takeoff(target_altitude)
    sleep(5)
     #   sleep(0.1) # need 10hz freq 
    #while current_altitude > 0.1:
    node.land()
    
    node.arm_drone(False)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



