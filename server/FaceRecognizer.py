#!/usr/bin/env python
import argparse, pickle, warnings
import numpy as np, cv2, dlib
import rospy
from sensor_msgs.msg import CompressedImage
from sklearn.pipeline import Pipeline
from sklearn.lda import LDA
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC
from sklearn.grid_search import GridSearchCV
from sklearn.mixture import GMM
from sklearn.tree import DecisionTreeClassifier
from sklearn.naive_bayes import GaussianNB
import openface
import comm

def getImages(path):
    with open(path, 'rb') as f:
        pickle.load(f)
        imgs = pickle.load(f)
        if imgs is None:
            print 'cannot load file: {}'.format(path)
            return None
        return imgs

def getData():
    imgs = getImages(pklpath)  # /home/anchore/faces.pkl
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
#        numIdentities = len(set(y + [-1]))
#        if numIdentities <= 1:
#            return

    if classifier == 'LinearSvm':
        clf = SVC(C=1, kernel='linear', probability=True)
    elif classifier == 'GridSearchSvm':
        '''
Warning: In our experiences, using a grid search over SVM hyper-parameters only
gives marginally better performance than a linear SVM with C=1 and
is not worth the extra computations of performing a grid search.
        '''
        param_grid = [
            {'C': [1, 10, 100, 1000],
             'kernel': ['linear']},
            {'C': [1, 10, 100, 1000],
             'gamma': [0.001, 0.0001],
             'kernel': ['rbf']}
        ]
        clf = GridSearchCV(SVC(C=1, probability=True), param_grid, cv=5)
    elif classifier == 'GMM':  # Doesn't work best
        clf = GMM(n_components=nClasses)

    # ref:
    # http://scikit-learn.org/stable/auto_examples/classification/plot_classifier_comparison.html#example-classification-plot-classifier-comparison-py
    elif classifier == 'RadialSvm':  # Radial Basis Function kernel
        # works better with C = 1 and gamma = 2
        clf = SVC(C=1, kernel='rbf', probability=True, gamma=2)
    elif classifier == 'DecisionTree':  # Doesn't work best
        clf = DecisionTreeClassifier(max_depth=20)
    elif classifier == 'GaussianNB':
        clf = GaussianNB()

    # ref: https://jessesw.com/Deep-Learning/
    elif classifier == 'DBN':
        from nolearn.dbn import DBN
        clf = DBN(
            [embeddings.shape[1], 500, labelsNum[-1:][0] + 1],  # i/p nodes, hidden nodes, o/p nodes
            learn_rates=0.3,
            # Smaller steps mean a possibly more accurate result, but the
            # training will take longer
            learn_rate_decays=0.9,
            # a factor the initial learning rate will be multiplied by
            # after each iteration of the training
            epochs=300,  # no of iternation
            # dropouts = 0.25, # Express the percentage of nodes that
            # will be randomly dropped as a decimal.
            verbose=1
        )

        if ldaDim > 0:
            clf_final = clf
            clf = Pipeline(
                [
                    ('lda', LDA(n_components=ldaDim)),
                    ('clf', clf_final)
                ]
        )

    return clf.fit(X, y)

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

    bb = align.getLargestFaceBoundingBox(npImg)
    if bb is None:
        print 'cannot find a face on frame {}'.format(rosData.header.frame_id)
        cv2.imshow('Face', npImg)
        cv2.waitKey(1000 // fps)
        return

    alignedFace = align.align(96, npImg, bb,
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

    p1 = (bb.left(), bb.top())
    p2 = (bb.right(), bb.bottom())
    cv2.rectangle(npImg, p1, p2, (0, 255, 0), 3)
#    for p in openface.AlignDlib.OUTER_EYES_AND_NOSE:
#        cv2.circle(npImg, center=landmarks[p], radius=3, color=(0, 255, 0), thickness=-1)
    cv2.putText(
        npImg,
        str(identity) + ':' + name,
        (bb.left(), bb.top() - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=0.75,
        color=(0, 255, 0),
        thickness=2
    )
    cv2.imshow('Face', npImg)
    print 'you\'re most likely {} on frame {} with confidence {:.2f}'.format(
        name, rosData.header.frame_id, confidence)
    cv2.waitKey(1000 // fps)

warnings.filterwarnings("ignore")

parser = argparse.ArgumentParser()
parser.add_argument('--pklpath', type=str, default='/home/anchore/faces.pkl')
parser.add_argument(
    '--classifier',
    type=str,
    choices=['LinearSvm', 'GridSearchSvm', 'GMM', 'RadialSvm', 'DecisionTree', 'GaussianNB', 'DBN'],
    help='The type of classifier to use.',
    default='GridSearchSvm'
)
parser.add_argument('--ldaDim', type=int, default=-1)
args = parser.parse_args()

align = comm.align
net = comm.net

pklpath = args.pklpath
classifier = args.classifier
ldaDim = args.ldaDim

le, clf = None, None

if __name__ == '__main__':
    fps = comm.FPS
    imgSize = comm.IMG_SIZE

    with open(pklpath, 'rb') as f:
        persons = pickle.load(f)
    le = LabelEncoder().fit(list(persons.keys()))
    clf = train()

    faceRecognize()