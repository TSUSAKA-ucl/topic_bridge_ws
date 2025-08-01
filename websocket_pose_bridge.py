import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import ssl
import websockets
import asyncio
import json
from threading import Thread

class PoseBridgeNode(Node):
    def __init__(self):
        super().__init__('websocket_pose_bridge')
        self.publisher_ = self.create_publisher(PoseStamped, 'input_pose', 10)
        self.subscription_ = self.create_subscription(
            PoseStamped,
            'output_pose',
            self.pose_callback,
            10
        )
        self.websocket_clients = set()
        self.get_logger().info("PoseBridgeNode initialized")

    def publish_pose(self, data):
        try:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"

            pos = data['position']
            ori = data['orientation']

            pose.pose.position.x = float(pos['x'])
            pose.pose.position.y = float(pos['y'])
            pose.pose.position.z = float(pos['z'])
            pose.pose.orientation.x = float(ori['x'])
            pose.pose.orientation.y = float(ori['y'])
            pose.pose.orientation.z = float(ori['z'])
            pose.pose.orientation.w = float(ori['w'])

            self.publisher_.publish(pose)
            self.get_logger().info("Published input_pose")
        except Exception as e:
            self.get_logger().error(f"Invalid input data: {e}")

    def pose_callback(self, msg: PoseStamped):
        # Convert PoseStamped to JSON
        message = {
            "position": {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z
            },
            "orientation": {
                "x": msg.pose.orientation.x,
                "y": msg.pose.orientation.y,
                "z": msg.pose.orientation.z,
                "w": msg.pose.orientation.w
            }
        }
        data_str = json.dumps(message)
        # Send to all connected websocket clients
        for client in list(self.websocket_clients):
            asyncio.run_coroutine_threadsafe(
                client.send(data_str), asyncio.get_event_loop()
            )


async def websocket_handler(websocket, node: PoseBridgeNode):
    node.websocket_clients.add(websocket)
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                node.publish_pose(data)
            except Exception as e:
                node.get_logger().error(f"WebSocket JSON error: {e}")
    finally:
        node.websocket_clients.remove(websocket)

# closure factory
def make_handler(node):
    async def handler(ws):
        await websocket_handler(ws, node)
    return handler

async def main():
    rclpy.init()
    node = PoseBridgeNode()

    # Run rclpy executor in separate thread
    def ros_spin():
        rclpy.spin(node)

    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # Start websocket server (asyncio)
    basedir = os.path.dirname(__file__)  # this directory
    certfile = os.path.join(basedir, "..", "certificates", "localhost.pem")
    keyfile = os.path.join(basedir, "..", "certificates", "localhost-key.pem")

    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(certfile=certfile, keyfile=keyfile)
    start_server = websockets.serve(
        make_handler(node),
        "0.0.0.0", 9090,
        ssl=ssl_context
    )

    loop = asyncio.get_event_loop()
    await start_server
    print("WebSocket server running at ws://0.0.0.0:9090")
    try:
        await asyncio.Future()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
