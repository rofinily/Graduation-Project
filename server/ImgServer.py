#!/usr/bin/env python
import sys, json, argparse
import txaio
from autobahn.twisted.websocket import WebSocketServerProtocol, \
    WebSocketServerFactory
from twisted.python import log
from twisted.internet import reactor
# import Util4WebServer as uws
from Util4WebServer import MSG, rawFace

parser = argparse.ArgumentParser()
parser.add_argument('--port', type=int, default=9001, help='WebSocket Port')
args = parser.parse_args()

def startServer():
    factory = WebSocketServerFactory("ws://localhost:{}".format(port))
    factory.protocol = MyServerProtocol

    reactor.listenTCP(port, factory)
    reactor.run()

class MyServerProtocol(WebSocketServerProtocol):
    def __init__(self):
        pass

    def onConnect(self, request):
        print("Client connecting: {}".format(request.peer))

    def onOpen(self):
        print("WebSocket connection open.")

    def onMessage(self, payload, isBinary):
        raw = payload.decode('utf8')
        j = json.loads(raw)
        
        if j['cmd'] == 'nothing':
            pass
        elif j['cmd'] == 'raw':
            msg = rawFace()
            self.sendMessage(json.dumps(msg))
        else:
            pass

    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {}".format(reason))

if __name__ == '__main__':
    txaio.use_twisted()
    log.startLogging(sys.stdout)

    port = args.port
    startServer()