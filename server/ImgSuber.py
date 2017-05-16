#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, comm, numpy as np, cv2, fcntl
from sensor_msgs.msg import CompressedImage
import json, pickle
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
    msg['content'] = unicode(rosData.data, errors='ignore')
    # print(rosData.data)
    ws.send(json.dumps(msg))

    print 'frame {} sent'.format(rosData.header.frame_id)

if __name__ == '__main__':

    imgSize = comm.IMG_SIZE
    try:
        ws = create_connection('ws://localhost:9000/')
        subImg()
    finally:
        ws.close()