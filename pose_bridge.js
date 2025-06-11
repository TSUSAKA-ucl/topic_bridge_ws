const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');

const wss = new WebSocket.Server({ port: 9090 });  // WebSocket ポート

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('pose_bridge');
  const pub = node.createPublisher('geometry_msgs/msg/PoseStamped', 'controller_pose');

  wss.on('connection', (ws) => {
    console.log('WebSocket connected');

    ws.on('message', (message) => {
      try {
        const data = JSON.parse(message);
	const ts = data.timestamp;
	// const sec = Math.floor(ts/1000);
	// const nanosec = Math.floor((ts % 1000)*1e6);
	const sec = BigInt(Math.floor(ts/1000));
	const nanosec = BigInt(Math.floor((ts % 1000)*1e6));
	// 型を確認
	// console.log('sec:', sec, typeof sec);
	// console.log('nanosec:', nanosec, typeof nanosec);
	//
	const timeStamp = new rclnodejs.Time(sec,nanosec)
        const msg = {
          header: {
            stamp: timeStamp,
            frame_id: 'world',
          },
          pose: {
            position: data.position,
            orientation: data.orientation,
          },
        };
	// console.log('msg: ', msg);
        pub.publish(msg);
      } catch (err) {
        console.error('Invalid message:', err);
      }
    });
  });

  rclnodejs.spin(node);
  console.log('WebSocket server listening on ws://localhost:9090');
});
