# Pose Bridge WebServer: `rclnodejs`を使ったWebSocketサーバーのテスト

1. [`for-rclnodejs.html`](./for-rclnodejs.html)のある場所で
   HTTP serverを、`python3 -m http.server 8080`で立ち上げる
2. [`runme.sh`](./runme.sh)(`node pose_bridge.js`)でWebSocketサーバーを立ち上げる  
   最初は、ここで`npm install`で依存関係をインストールしておく
3. ブラウザで`http:localhost:8080/for-rclnodejs.html`を見る
4. F12でブラウザのDevToolsを出し、immersive web emulatorのWebXR画面を出して操作する
5. 別の端末で `ros2 topic echo /controller_pose`でROSのトピックを見る

`rclnodejs`の何処かでタイムスタンプが消えてしまっているが当面気にしない
