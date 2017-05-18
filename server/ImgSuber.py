#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, comm, base64 as b64
from sensor_msgs.msg import CompressedImage
import json
from websocket import create_connection

MSG = {
    'SET_TMP_IMG': {
        'cmd': 'setTmpImg',
        'content': None
    }
}

def subImg():
    rospy.init_node('faceHandler', anonymous=True)
    rospy.Subscriber("imgTransporter", CompressedImage, callback, queue_size=1, buff_size=imgSize)
    print 'suber is ready'
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def callback(rosData):
    msg = MSG['SET_TMP_IMG']
    msg['content'] = b64.b64encode(rosData.data)
    # print(rosData.data)
    ws.send(json.dumps(msg))

    print 'frame {} sent'.format(rosData.header.frame_id)

if __name__ == '__main__':

    imgSize = comm.IMG_SIZE
    try:
        ws = create_connection('ws://localhost:9001/')
        subImg()
    finally:
        ws.close()