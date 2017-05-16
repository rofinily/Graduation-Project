#!/usr/bin/env python
import rospy, comm, numpy as np, cv2, fcntl
from sensor_msgs.msg import CompressedImage

def saveImg():
    rospy.init_node('faceHandler', anonymous=True)
    rospy.Subscriber("imgTransporter", CompressedImage, callback, queue_size=1, buff_size=imgSize)
    print 'saver is ready'
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def callback(rosData):
#    npArr = np.fromstring(rosData.data, np.uint8)
#    npImg = cv2.imdecode(npArr, cv2.CV_LOAD_IMAGE_COLOR)
#    cv2.imwrite('tmp.jpg', npImg)
    with open('tmpImg', 'wb') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        f.write(rosData.data)
        fcntl.flock(f, fcntl.LOCK_UN)
    print 'frame {} saved'.format(rosData.header.frame_id)

if __name__ == '__main__':
    imgSize = comm.IMG_SIZE

    saveImg()