import os
import sys
import importlib
import signal

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

import ssl
import websockets
import asyncio
import json
import msgpack
from threading import Thread
_do_shutdown = True # False if shutdown is in progress

class TopicBridgeNode(Node):
    def __init__(self, loop, initial_pub, initial_sub):
        self.loop = loop
        super().__init__('websocket_topic_bridge')
        # #### initial_pub & initial_sub are defined in default_topics.py
        # initial_pub = {'actuator1':Actuators, 'time1':Time, }
        # initial_sub = {'output_pose':PoseStamped, 'string1':String, }
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
                data['header']['stamp']['nanosec'] = (timestamp % 1000) * 1000 * 1000
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
                if isinstance(binary, str) :
                    node.get_logger().info(f'socket receive STRING: {binary}')
                data = msgpack.unpackb(binary)
                node.publish_topic(data)
            except Exception as e:
                node.get_logger().warning(f"WebSocket MessagePack error: {e}")
    except websockets.ConnectionClosed:
        node.get_logger().warning("Connection closed. Waiting to reconnecting...")
    finally:
        node.websocket_clients.remove(websocket)

# closure factory
def make_handler(node):
    async def handler(ws):
        await websocket_handler(ws, node)
    return handler

def make_shutdown_handler(ros_thread, loop):
    def shutdown(signal_number, frame):
        global _do_shutdown
        if _do_shutdown :
            _do_shutdown = False
            rclpy.shutdown()
            for task in asyncio.all_tasks(loop):
                task.cancel()
            loop.call_soon_threadsafe(loop.stop)
            ros_thread.join()
    return shutdown

async def main(port, loop, pub, sub):
    rclpy.init()
    node = TopicBridgeNode(loop, pub, sub)

    # Run rclpy executor in separate thread
    def ros_spin():
        rclpy.spin(node)
    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    shutdown = make_shutdown_handler(ros_thread, loop)
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Start websocket server (asyncio)
    basedir = os.path.dirname(__file__)  # this directory
    certfile = os.path.join(basedir, "..", "certificates", "cert.pem")
    keyfile = os.path.join(basedir, "..", "certificates", "key.pem")

    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(certfile=certfile, keyfile=keyfile)
    start_server = websockets.serve(
        make_handler(node),
        "0.0.0.0", port,
        ssl=ssl_context
    )

    await start_server
    node.get_logger().info("WebSocket server running at wss://0.0.0.0:"+str(port))
    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) >= 3 :
        port = int(sys.argv[2])
    else:
        port = 9090
    if len(sys.argv) >= 2 :
        topic_list_file = sys.argv[1]
    else:
        topic_list_file = 'default_topics.py'
    if topic_list_file.endswith('.py') :
        topic_list_file = topic_list_file.removesuffix('.py')
    topic_list = importlib.import_module(topic_list_file)

    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        # If this thread doesn't have a event_loop, create a new one.
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
    # loop = asyncio.get_event_loop()
    loop.run_until_complete(main(port, loop,
                                 topic_list.initial_pub,
                                 topic_list.initial_sub ))
