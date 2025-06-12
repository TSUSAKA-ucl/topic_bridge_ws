const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');

const wss = new WebSocket.Server({ port: 9090 });  // WebSocket ポート

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('pose_bridge');
  // node.declareParameter({name: 'base_name', value: 'base_link'});
  node.declareParameter(new rclnodejs.Parameter('base_name', rclnodejs.ParameterType.PARAMETER_STRING, 'base_link'));
  const pubR = node.createPublisher('geometry_msgs/msg/PoseStamped', 'right_controller_pose');
  const pubL = node.createPublisher('geometry_msgs/msg/PoseStamped', 'left_controller_pose');

  wss.on('connection', (ws) => {
    console.log('WebSocket connected');
    // WebSocket からのメッセージ受信
    ws.on('message', (message) => {
      try {
        const data = JSON.parse(message);
	const ts = data.timestamp;
	const sec = Math.floor(ts/1000);
	const nanosec = Math.floor((ts % 1000)*1e6);
	// const sec = BigInt(Math.floor(ts/1000));
	// const nanosec = BigInt(Math.floor((ts % 1000)*1e6));
	// const timeStamp = new rclnodejs.Time(sec,nanosec,
	// 				     rclnodejs.Clock.ClockType.ROS_TIME)
	const frameID = node.getParameter('base_name').value;
        const msg = {
          header: {
            stamp: {
	      sec: sec,
	      nanosec: nanosec,
	    },
            frame_id: frameID,
          },
          pose: {
            position: data.position,
            orientation: data.orientation,
          },
        };
	// console.log('msg: ', msg);
	if (data.hand == 'right') {
	  pubR.publish(msg);
	} else if (data.hand == 'left') {
	  pubL.publish(msg);
	} else {
	  console.error('Unknown hand:', data.hand);
	}
      } catch (err) {
        console.error('Invalid message:', err);
      }
    });
  });

  rclnodejs.spin(node);
  console.log('WebSocket server listening on ws://localhost:9090');
});
