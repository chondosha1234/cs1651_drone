import rclpy
from rclpy.node import Node

import asyncio
from mavsdk import System

from std_msgs.msg import String
from std_srvs.srv import Trigger


class MavsdkNode(Node):
    def __init__(self):
        super().__init__('mavsdk_node')
        self.subscription = self.create_subscription(
                String,
                'mavsdk_topic',
                self.command_callback,
                10)
        self.drone = System(mavsdk_server_address="localhost")
        self.get_logger().info("Connecting to drone...")
        #rclpy.create_task(self.connect_to_drone())
        #await self.connect_to_drone()
        self.srv = self.create_service(Trigger, "connect", self.connect_callback)
        self.loop = asyncio.get_event_loop()
        self.get_logger().info("end init function")        

    async def connect_to_drone(self):
        self.get_logger().info("trying to connect to drone...")
        await self.drone.connect()   # make sure mavsdk server is running with right comms
        self.get_logger().info("Connected to drone.")

    def connect_callback(self, request, response):
        self.get_logger().info("inside connect callback")
        self.loop.create_task(self.connect_to_drone)
        response.success = True
        response.message = "connect call_back complete"
        return response

    def command_callback(self, msg):
        self.get_logger().info(f"received command: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    mavsdk_node = MavsdkNode()
    try:
        rclpy.spin(mavsdk_node)
    
    finally:
        mavsdk_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
