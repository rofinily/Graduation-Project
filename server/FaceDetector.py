#!/usr/bin/env python
import time
import numpy as np, dlib, cv2
import rospy
from sensor_msgs.msg import CompressedImage
import comm

align = comm.align

def faceDetect():
    rospy.init_node('faceDetector', anonymous=True)
    rospy.Subscriber("imgTransporter", CompressedImage, callback, queue_size=1, buff_size=imgSize)
    print 'detector is ready'
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def callback(rosData):
    npArr = np.fromstring(rosData.data, np.uint8)
    npImg = cv2.imdecode(npArr, cv2.CV_LOAD_IMAGE_COLOR)
    markAllFacesByDlib(npImg)
    cv2.imshow('Face Detector', npImg)
    print 'frame {} marked'.format(rosData.header.frame_id)
    cv2.waitKey(1000 // fps)

#def getAllFacesByDlib(rgbImg):
#    assert rgbImg is not None
#    faces = []
#    try:
#        faces = detector(rgbImg, 1)
#    except Exception as e:
#        print("Warning: {}".format(e))
#    finally:
#        return faces

#def markLargestFaceByDlib(rgbImg):
#    assert rgbImg is not None
#    faces = detector(rgbImg, 1)
#    if len(faces) == 0:
#        print "Unable to find the largest face"
#        return rgbImg
#    print 'found the largest face'
#    largestFace = max(faces, key=lambda rect: rect.width() * rect.height())
#    p1 = (largestFace.left(), largestFace.top())
#    p2 = (largestFace.right(), largestFace.bottom())
#    cv2.rectangle(rgbImg, p1, p2, (0, 255, 0), 3)
#    return rgbImg

def getAllFaceBoxes(npImg, detector='dlib'):
    if detector == 'dlib':
        return align.getAllFaceBoundingBoxes(npImg)
#    elif detector == 'opencv':
#        pass
    else:
        raise Exception('no such detector: {}'.format(detector))

def markAllFacesByDlib(npImg):
    faces = align.getAllFaceBoundingBoxes(npImg)
    # print 'found {} face'.format(len(faces))
    for face in faces:
        p1, p2 = (face.left(), face.top()), (face.right(), face.bottom())
        cv2.rectangle(npImg, p1, p2, (0, 255, 0), 2)
    if len(faces) == 0:
        return False
    return True

def markAllFacesByOpenCV(img):
    assert img is not None
    gray = img
    if img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = haar.detectMultiScale(gray, 1.2, 5)
    print 'found {} face'.format(len(faces))
    for (x, y, w, h) in faces:
        p1, p2 = (x, y), (x + w, y + h)
        cv2.rectangle(img, p1, p2, (0, 255, 0), 2)
    if len(faces) == 0:
        return False
    return True

if __name__ == '__main__':
    fps = comm.FPS
    imgSize = comm.IMG_SIZE
    #detector = dlib.get_frontal_face_detector()
    #haar = cv2.CascadeClassifier( \
    #    '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')
    
    faceDetect()