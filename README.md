# Pose Bridge WebServer: `rclnodejs`を使ったWebSocketサーバーのテスト

1. [`for-rclnodejs.html`](./for-rclnodejs.html)のある場所で
   HTTP serverを、`python3 -m http.server 8080`で立ち上げる
2. [`runme.sh`](./runme.sh)(`node pose_bridge.js`)で
   WebSocketサーバーを立ち上げる  
   最初は、ここで`npm install`で依存関係をインストールしておく
3. ブラウザで`http:localhost:8080/for-rclnodejs.html`を見る
4. F12でブラウザのDevToolsを出し、immersive web emulatorのWebXR画面を出して操作する
5. 別の端末で `ros2 topic echo /right_controller_pose`でROSのトピックを見る

# Python版ブリッジ: httpsおよびwss使用
websocketサーバーは`websocket_topic_bridge.py`. 
httpsサーバーは`server.sh`(シバン付きpython).
ブラウザで開くべきHTMLは`for-rclpy.html`

## `websocket_topic_bridge.py`のnodejs版との違い

nodejs版と互換性は無い
1. ssl.SSLContextでcertfileとkeyfileを使い、wssで接続する
2. ROS2のpublisherとsubscriberの両対応(subscriberは未完成)
3. webSocketは、MessagePackを使って、jsonでなくbinaryで通信
4. topic名タグ(`'topic'`)を使って、複数トピック
   複数タイプ(PoseStamped, Actuators)に対応

オブジェクト(トピック)のシリアライズ・デシリアライズは**jsonでなく
**MessagePackを使用している。そのため
[`https://unpkg.com/@msgpack/msgpack@3.1.2/dist.umd/msgpack.min.js`](https://unpkg.com/@msgpack/msgpack@3.1.2/dist.umd/msgpack.min.js)
をブラウザに取り込んでおく必要がある。Next.jsの場合は
`@msgpack/msgpack`をinstallしimportすれば良い

サーバー側(Python)は、`pip install msgpack`で良い
