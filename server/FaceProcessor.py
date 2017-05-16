#!/usr/bin/env python
import argparse, imagehash, pickle, os
from PIL import Image
import numpy as np, cv2, dlib
import rospy
from sensor_msgs.msg import CompressedImage
import openface
import comm

parser = argparse.ArgumentParser()
parser.add_argument('--name', type=str, help="Your Name", default='unknown')
parser.add_argument('--identity', type=int, help="Your ID", default=-1)
parser.add_argument('--pklpath', type=str, default='/home/anchore/faces.pkl')
args = parser.parse_args()

align = comm.align
net = comm.net

images = {}
persons = {}

def faceProcess():
    rospy.init_node('faceProcessor', anonymous=True)
    rospy.Subscriber("imgTransporter", CompressedImage, callback, queue_size=1, buff_size=imgSize)
    print 'processor is ready'
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def callback(rosData):
    npArr = np.fromstring(rosData.data, np.uint8)
    npImg = cv2.imdecode(npArr, cv2.CV_LOAD_IMAGE_COLOR)

    bb = align.getLargestFaceBoundingBox(npImg)
    if bb is None:
        print 'cannot find a face on frame {}'.format(rosData.header.frame_id)
        cv2.imshow('Face', npImg)
        cv2.waitKey(1000 // fps)
        return
    p1 = (bb.left(), bb.top())
    p2 = (bb.right(), bb.bottom())
    cv2.rectangle(npImg, p1, p2, (0, 255, 0), 3)
    cv2.imshow('Face', npImg)
    cv2.waitKey(1000 // fps)

    alignedFace = align.align(96, npImg, bb,
        landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
    phash = str(imagehash.phash(Image.fromarray(alignedFace)))
    if phash not in images:
        rep = net.forward(alignedFace)
        images[phash] = {
            'identity' : identity,
            'rep' : rep
        }
        print 'got representation of {}\'s face on frame {}'.format(
            name, rosData.header.frame_id)
    else:
        print 'frame already proceeded'

def initData():
    if os.path.exists(pklpath):
        with open(pklpath, 'rb') as f:
            persns = pickle.load(f)
            persons.update(persns)
            imgs = pickle.load(f)
            images.update(imgs)
            print 'pkl file already read'
    persons[identity] = name

    if -1 in persons:
        persons.pop(-1)

def saveData():
    with open(pklpath, 'wb') as f:
        pickle.dump(persons, f)
        pickle.dump(images, f)
        print 'pkl file already written'

if __name__ == '__main__':
    fps = comm.FPS
    imgSize = comm.IMG_SIZE

    name = args.name
    identity = args.identity
    pklpath = args.pklpath

    try:
        initData()
        faceProcess()
    finally:
        saveData()