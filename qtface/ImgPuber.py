#!/usr/bin/env python

import numpy as np, cv2, rospy, time, argparse, comm
from sensor_msgs.msg import CompressedImage

parser = argparse.ArgumentParser()
parser.add_argument('--camid', type=int, default=0, help='Video Camera ID')
args = parser.parse_args()

def imgPub():
    rospy.init_node('imgPuber', anonymous=True)
    imgPuber = rospy.Publisher('imgTransporter', CompressedImage, latch=True, queue_size=1)
    pubRate = rospy.Rate(fps)
    count = 1
    while not rospy.is_shutdown():
        ret, img = capture.read()
        msg = CompressedImage()
        msg.header.frame_id = str(count)
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        imgPuber.publish(msg)
        print 'frame %d published' % count
        count += 1
        pubRate.sleep()
        #time.sleep(1.3)

#def scaledImg(img, scale=0.5):
#    assert img is not None
#    sp = img.shape
#    h, w = sp[0], sp[1]
#    h = int(round(h * scale, 0))
#    w = int(round(w * scale, 0))
#    return cv2.resize(img, (w, h))

if __name__ == '__main__':
    camid = args.camid

    while True:
        capture = cv2.VideoCapture(camid)
        if capture.isOpened():
            print 'Camera {} now available'.format(camid)
            break
        else:
            print 'Camera {} unavailable, will require camera in 1 second'.format(camid)
            time.sleep(1)

    fps = comm.FPS

    try:
        imgPub()
    except rospy.ROSInterruptException as e:
        print e
    finally:
        capture.release()
