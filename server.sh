#!/usr/bin/env python3
#
#!/bin/bash
# python3 -m http.server 8080
#
# https_server.py
import os
import http.server
import ssl

HOST = 'localhost'
PORT = 4443

# 任意のディレクトリをルートにする
Handler = http.server.SimpleHTTPRequestHandler

httpd = http.server.HTTPServer(('', PORT), Handler)

basedir = os.path.dirname(__file__)  # this directory
certfile = os.path.join(basedir, "..", "certificates", "cert.pem")
keyfile = os.path.join(basedir, "..", "certificates", "key.pem")

context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(certfile=certfile, keyfile=keyfile)
# 証明書と秘密鍵を読み込んで SSL 化
httpd.socket = context.wrap_socket(httpd.socket,
				   server_side=True,)

print(f"Serving HTTPS on https://{HOST}:{PORT}")
httpd.serve_forever()
