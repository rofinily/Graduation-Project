#!/usr/bin/env python
import sys, json, comm, argparse, base64 as b64
import txaio
from autobahn.twisted.websocket import WebSocketServerProtocol, \
    WebSocketServerFactory
from twisted.python import log
from twisted.internet import reactor
import Util4WebServer as uws
from Util4WebServer import MSG

log.startLogging(sys.stdout)

parser = argparse.ArgumentParser()
parser.add_argument('--port', type=int, default=9000, help='WebSocket Port')
args = parser.parse_args()

port = args.port

def startServer():
    factory = WebSocketServerFactory('ws://localhost:{}'.format(port))
    factory.protocol = MyServerProtocol

    reactor.listenTCP(port, factory)
    reactor.run()

class MyServerProtocol(WebSocketServerProtocol):
    def __init__(self):
        uws.initData()
        uws.updateRecognizer()

    def onConnect(self, request):
        print('Client connecting: {}'.format(request.peer))

    def onOpen(self):
        print('WebSocket connection open.')

    def onMessage(self, payload, isBinary):
        raw = payload.decode('utf8')
#        print(isBinary, type(payload))
        j = json.loads(raw)

        if j['cmd'] != 'raw':
            print(j['cmd'])
        
        if j['cmd'] == 'nothing':
            self.sendMessage(json.dumps(MSG['EMPTY']))
            
        elif j['cmd'] == 'raw':
            msg = uws.rawFace()
            self.sendMessage(json.dumps(msg))

        elif j['cmd'] == 'detect':
            msg = uws.detectFace()
            self.sendMessage(json.dumps(msg))

        elif j['cmd'] == 'train':
            msg = uws.trainFace(j['name'], int(j['identity']))
            self.sendMessage(json.dumps(msg))

        elif j['cmd'] == 'stopTrain':
            uws.saveData()
            uws.updateRecognizer()
            self.sendMessage(json.dumps(MSG['TRAIN_STOPED']))

        elif j['cmd'] == 'recognize':
            msg = uws.recognizeFace()
            self.sendMessage(json.dumps(msg))

        elif j['cmd'] == 'getPersons':
            msg = MSG['PERSONS']
            msg['content'] = uws.getPersons()
            # print(msg['content'])
            self.sendMessage(json.dumps(msg))

        elif j['cmd'] == 'uploadPic':
            print(j['name'], j['identity'])
            msg = uws.trainPic(j)
            self.sendMessage(json.dumps(msg))

        elif j['cmd'] == 'allFileUploaded':
            uws.saveData()
            uws.updateRecognizer()
            self.sendMessage(json.dumps(MSG['ALL_PIC_TRAINED']))

        elif j['cmd'] == 'delPerson':
            identity = int(j['identity'])
            uws.delPerson(identity)
            uws.saveData()
            uws.updateRecognizer()

            msg = MSG['PERSON_DELETED']
            msg['identity'] = identity
            self.sendMessage(json.dumps(msg))
        else:
            pass

    def onClose(self, wasClean, code, reason):
        print('WebSocket connection closed: {}'.format(reason))

if __name__ == '__main__':
    try:
        txaio.use_twisted()
        startServer()
    finally:
        uws.saveData()
