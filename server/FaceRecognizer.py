#!/usr/bin/env python
import argparse, pickle, warnings
import numpy as np, cv2
import rospy
from sensor_msgs.msg import CompressedImage
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC
from sklearn.grid_search import GridSearchCV
import openface
import comm

warnings.filterwarnings("ignore")

parser = argparse.ArgumentParser()
parser.add_argument('--pklpath', type=str, default='/home/anchore/faces.pkl')
args = parser.parse_args()

align = comm.align
net = comm.net

pklpath = args.pklpath

le, clf = None, None

def getImages(path):
    with open(path, 'rb') as f:
        pickle.load(f)
        imgs = pickle.load(f)
        if imgs is None:
            print 'cannot load file: {}'.format(path)
            return None
        return imgs

def getData():
    imgs = getImages(pklpath)
    if imgs is None:
        return None
    X = []
    y = []
    for img in imgs.values():
        X.append(img['rep'])
        y.append(img['identity'])
    numIdentities = len(set(y))
    if numIdentities == 0:
        print 'no persons'
        return None
    X = np.vstack(X)
    y = np.array(y)
    return (X, y)

def train():
    d = getData()
    if d is None:
        return None

    (X, y) = d
    param_grid = [
        {
            'C': [1, 10, 100, 1000],
            'kernel': ['linear']
        },
        {
            'C': [1, 10, 100, 1000],
            'gamma': [0.001, 0.0001],
            'kernel': ['rbf']
        }
    ]

    global clf

    clf = GridSearchCV(SVC(C=1, probability=True), param_grid, cv=5)

    return clf.fit(X, y)

def updateRecognizer():
    persons = FaceProcessor.getPersons()
    if len(persons) <= 1:
        return;

    global le, clf
    le = LabelEncoder().fit(
        list(persons.keys())
    )
    clf = train()

def recognize(npImg):
    if clf is None:
        return None

    if npImg is None:
        return None

    npImg = comm.scaledImg(npImg)
    box = align.getLargestFaceBoundingBox(npImg)
    if box is None:
        return None

    persons = FaceProcessor.getPersons()

    alignedFace = align.align(96, npImg, box,
        landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
    rep = net.forward(alignedFace)
    rep1 = rep.reshape(1, -1)
    predictions = clf.predict_proba(rep).ravel()
    maxI = np.argmax(predictions)
    confidence = predictions[maxI]

    identity = le.inverse_transform(maxI)
    name = persons[identity]
    print(name, confidence)

    if confidence < 0.7:
        identity, name = -1, 'unknown'

    msg = {
        'identity': identity,
        'name': name,
        'confidence': confidence,
        'box': box
    }
    return msg

def faceRecognize():
    if clf is None:
        print 'failed to train'
        return
    rospy.init_node('faceRecognizer', anonymous=True)
    rospy.Subscriber("imgTransporter", CompressedImage, callback, queue_size=1, buff_size=imgSize)
    print 'recognizer is ready'
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def callback(rosData):
    npArr = np.fromstring(rosData.data, np.uint8)
    npImg = cv2.imdecode(npArr, cv2.CV_LOAD_IMAGE_COLOR)

    box = align.getLargestFaceBoundingBox(npImg)
    if box is None:
        print 'cannot find a face on frame {}'.format(rosData.header.frame_id)
        cv2.imshow('Face', npImg)
        cv2.waitKey(1000 // fps)
        return

    alignedFace = align.align(96, npImg, box,
        landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
    rep = net.forward(alignedFace)
    rep1 = rep.reshape(1, -1)
    predictions = clf.predict_proba(rep).ravel()
    maxI = np.argmax(predictions)
    confidence = predictions[maxI]
    identity, name = -1, 'Unknown'
    if confidence >= 0.9:
        identity = le.inverse_transform(maxI)
        name = persons[identity]

    #identity = svm.predict(rep)[0]

    p1 = (box.left(), box.top())
    p2 = (box.right(), box.bottom())
    cv2.rectangle(npImg, p1, p2, (0, 255, 0), 3)
#    for p in openface.AlignDlib.OUTER_EYES_AND_NOSE:
#        cv2.circle(npImg, center=landmarks[p], radius=3, color=(0, 255, 0), thickness=-1)
    cv2.putText(
        npImg,
        str(identity) + ':' + name,
        (box.left(), box.top() - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=0.75,
        color=(0, 255, 0),
        thickness=2
    )
    cv2.imshow('Face', npImg)
    print 'you\'re most likely {} on frame {} with confidence {:.2f}'.format(
        name, rosData.header.frame_id, confidence)
    cv2.waitKey(1000 // fps)

if __name__ == '__main__':
    fps = comm.FPS
    imgSize = comm.IMG_SIZE

    with open(pklpath, 'rb') as f:
        persons = pickle.load(f)
    le = LabelEncoder().fit(list(persons.keys()))
    clf = train()

    faceRecognize()