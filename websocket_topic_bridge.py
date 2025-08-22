import os

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

from geometry_msgs.msg import PoseStamped
from actuator_msgs.msg import Actuators
from std_msgs.msg import String
from builtin_interfaces.msg import Time


import ssl
import websockets
import asyncio
import json
import msgpack
from threading import Thread

class TopicBridgeNode(Node):
    def __init__(self, loop):
        self.loop = loop
        super().__init__('websocket_topic_bridge')
        initial_pub = {'pose1':PoseStamped,
                       'pose2':PoseStamped,
                       'pose3':PoseStamped,
                       'actuator1':Actuators,
                       'actuator2':Actuators,
                       'time1':Time,
                       'time2':Time,
                       'time3':Time
                       }
        initial_sub = {'output_pose':PoseStamped,
                       'joint1':Actuators,
                       'string1':String,
                       'string2':String,
                       }
        self.publishers_ = {}
        for topic in initial_pub.keys():
            publisher = self.create_publisher(initial_pub[topic], topic, 10)
            self.publishers_[topic] = publisher
        self.subscribers_ = {}
        for topic in initial_sub.keys():
            subscription = self.create_subscription(
                initial_sub[topic], topic,
                lambda msg, name=topic: self.sub_callback(msg, name),
                10
            )
            self.subscribers_[topic] = subscription
        self.websocket_clients = set()
        self.get_logger().info("TopicBridgeNode initialized")

    def publish_topic(self, data):
        try:
            topic_name = data.pop('topic')
            publisher = self.publishers_[topic_name]
            type = publisher.msg_type
        except KeyError:
            self.get_logger().error(f"Unknown topic: {data.get('topic', 'NO_TOPIC')}")
            return
        else:
            try:
                timestamp = int(data.pop('javascriptStamp',
                                         self.get_clock().now().nanoseconds / 1000))
                # self.get_logger().info('data: '+json.dumps(data))
                data['header']['stamp'] = {}
                data['header']['stamp']['sec'] = timestamp // 1000
                data['header']['stamp']['nanosec'] = (timestamp % 1000) * 1000
                msg = type()
                # msg = None
                # if type == PoseStamped:
                #     msg = PoseStamped()
                # else:
                #     self.get_logger().warning(f"Unknown topic name: {topic_name} with type {type}")
                if msg is not None:
                    # self.get_logger().info('before data: '+json.dumps(data))
                    set_message_fields(msg, data)
                    publisher.publish(msg)
                    # self.get_logger().info(f"Published to {topic_name}")
            except Exception as e:
                self.get_logger().error(f"Invalid input data: {e}")

    def sub_callback(self, msg, topic_name):
        # Convert ROS2 message to dit
        msg_dict = message_to_ordereddict(msg)
        # self.get_logger().info('Converted to dict: %s' % msg_dict)
        msg_dict['topic'] = topic_name
        # data_str = json.dumps(msg_dict)
        binary = msgpack.packb(msg_dict, use_bin_type=True)
        # Send to all connected websocket clients
        for client in list(self.websocket_clients):
            asyncio.run_coroutine_threadsafe(
                client.send(binary), self.loop
            )


async def websocket_handler(websocket, node: TopicBridgeNode):
    node.websocket_clients.add(websocket)
    try:
        async for binary in websocket:
            try:
                data = msgpack.unpackb(binary)
                node.publish_topic(data)
            except Exception as e:
                node.get_logger().error(f"WebSocket MessagePack error: {e}")
    finally:
        node.websocket_clients.remove(websocket)

# closure factory
def make_handler(node):
    async def handler(ws):
        await websocket_handler(ws, node)
    return handler

async def main(port, loop):
    rclpy.init()
    node = TopicBridgeNode(loop)

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
        "0.0.0.0", port,
        ssl=ssl_context
    )

    await start_server
    print("WebSocket server running at wss://0.0.0.0:"+str(port))
    try:
        await asyncio.Future()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    port = 9090
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main(port, loop))
