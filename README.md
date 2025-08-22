# WebSocket経由のROS2トピックのpub/sub
[rclpy(ROS2)のインストール](https://docs.ros.org/en/jazzy/Installation.html)に
関しては説明略。現状jazzy以外の動作確認はしていない

## (参考)Pose Bridge WebServer: `rclnodejs`を使ったWebSocketサーバーのテスト

1. [`for-rclnodejs.html`](./for-rclnodejs.html)のある場所で
   HTTP serverを、`python3 -m http.server 8080`で立ち上げる
2. [`runme.sh`](./runme.sh)(`node pose_bridge.js`)で
   WebSocketサーバーを立ち上げる  
   最初は、ここで`npm install`で依存関係をインストールしておく
3. ブラウザで`http:localhost:8080/for-rclnodejs.html`を見る
4. F12でブラウザのDevToolsを出し、immersive web emulatorのWebXR画面を出して操作する
5. 別の端末で `ros2 topic echo /right_controller_pose`でROSのトピックを見る

## Python版ブリッジ: httpsおよびwss使用
websocketサーバーは[`websocket_topic_bridge.py`](
websocket_topic_bridge.py). 
httpsサーバーは[`server.sh`](server.sh)(シバン付きpython).
ブラウザで開くべきテスト用HTMLは[`for-rclpy.html`](for-rclpy.html)

### `websocket_topic_bridge.py`のnodejs版との違い

nodejs版と互換性は無い
1. `ssl.SSLContext`でcertfileとkeyfileを使い、wssで接続する
2. ROS2のpublisherとsubscriberの両対応
3. webSocketは、MessagePackを使って、jsonでなくbinaryで通信
4. 所定のメッセージ型をimportし、topic名をキーにしてリストに追加する
   だけで、任意の型のpublisher/subscriberになる
5. topic名タグ(`'topic'`)を使って、複数トピック(任意)
   複数タイプ(PoseStamped, Actuators, Stringなどimportしたもの)に対応
6. headerは空のオブジェクト{}に対応。その場合stampはbridgeが付ける

オブジェクト(トピックメッセージ)のシリアライズ・デシリアライズは
**jsonでなく**MessagePackを使用しているため
[`https://unpkg.com/@msgpack/msgpack@3.1.2/dist.umd/msgpack.min.js`](
https://unpkg.com/@msgpack/msgpack@3.1.2/dist.umd/msgpack.min.js)
をブラウザに取り込んでおく必要がある。[`for-rclpy.html`](for-rclpy.html)用の
`msgpack`は
[`install-msgpack.min.js.sh`](./install-msgpack.min.js.sh)で
このディレクトリの`./js/`の下に持ってくることができる。  
Next.jsの場合は`@msgpack/msgpack`をinstallしimportすれば良い。webWorkerの
場合はimportしてVite等でビルド(bind)するのが簡単  
サーバー側(Python)は、`pip install msgpack`だけで良い

#### 立ち上げ方と使い方
##### 立ち上げ方
ros2(rclpy)およびpythonの`msgpack`がインストール済の環境で
```
python3 websocket_topic_bridge.py
```
と実行するだけ。

現状、プロトコルはwssポート番号は9090に固定。
wssのためのcertファイルとkeyファイルは(オレオレ認証局なので)、このファイル
からの相対パス(`../certificates/localhost.pem`など)がベタ書きしてある
##### 使い方
jsからpublishするときは、IDL(ros2の.msgファイル)の定義に従ってオブジェクトを
作成し、さらに`topic`というキーでトピック名を付けて、それをMessagePack.encodeで
エンコードしてws.sendで送る。`javascriptStamp`というキーでjavascriptの
時刻(ミリ秒単位)を付与しておくと、`websocket_topic_bridge.py`が
メッセージのheaderのstampをその値に書き換えるため、`header`キーには
空のオブジェクト`{}`を付けておくだけでも利用できる。`topic`と`javascriptStamp`
キーは、pythonでros2メッセージにするときに取り除かれる。

jsでのsubscribeは, webSocketのonmessageになる。さらにBlob(ブラウザによる?)で
来るので、`arrayBuffer()`で変換しさらに`MessagePack.decode`でデコードする必要が
ある。`websocket_topic_bridge.py`によりメッセージの内容だけでなく`topic`と
いうスロットにトピック名が付与される。

#### 実装メモ
pythonのwebsocketサーバー(`import websockets`)は`asyncio`対応で、
わかりやすくメインスレッドで動かす。同時にros2 topicをsubscribeするため
ros2のspinnerはサブスレッドで動かし、subscriberがcallbackを呼んだ時
メインスレッドのコルーチンを起動する
(`asyncio.run_coroutine_threadsafe(...., self.loop)`、
`self.loop`はasyncioの`event_loop`)。

`self.publishers_`と`self.subscribers_`にトピック名をキーして
publisher/subscriptionをバリューにして登録することで、その
トピックのpublish/subscribeが可能になる。
現状、これらは`initial_pub`および`initial_sub`の辞書(トピック名:型)から
読み込んで生成するだけで、ダイナミックに生成する機能はつくっていない。

大きなメッセージをjavaScriptに送り込むことは無いであろうから、MessagePack
で送る必要があるかどうかは疑問(Blobで受け取るためpromiseが必要)。
実際にブラウザで受け取る場合はBlobになっている様子であり他の型の判定は
不要かもしれない(最初にBlobかどうかを調べるべき)。ごく小さな文字列等の
メッセージならばjson stringの方が(簡単で)良いかもしれないが、PoseStamped
程度になればMessagePackのほうが良くなるだろう。さらにjsでのencodeと
pythonでのdecodeは記述も簡単である。
