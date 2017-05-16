#!/usr/bin/env python
import json
from websocket import create_connection

ws = create_connection("ws://localhost:9000/")

msg = {
	'cmd': 'getPersons'
}

ws.send(json.dumps(msg))

print(ws.recv())

ws.close()