#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2, numpy as np, base64, imagehash, pickle
from PIL import Image
import openface
import comm, FaceDetector, FaceProcessor, FaceRecognizer

tmpImgPath = 'tmpImg'

MSG = {
    'PROCEEDED': {
        'type': 'proceeded',
        'detail': None,
        'content': None

    },
    'UPLOADED_SIZE': {
        'type': 'uploadedSize',
        'uploadedSize': None
    },
    'PERSONS': {
        'type': 'persons',
        'content': None
    },
    'ALL_PIC_TRAINED': {
        'type': 'allPicTrained'
    },
    'EMPTY': {
        'type': 'empty'
    }
}

if -1 in FaceProcessor.persons:
    FaceProcessor.persons.pop(-1)

def scaledImg(img, scale=0.5):
    sp = img.shape
    h, w = sp[0], sp[1]
    h = int(round(h * scale, 0))
    w = int(round(w * scale, 0))
    return cv2.resize(img, (w, h))

def rawFace():
    with open(tmpImgPath, 'rb') as f:
        npImg = comm.string2NpImg(f.read())

    if npImg is None:
        return MSG['EMPTY']

    npImg = scaledImg(npImg)

    content = comm.npImg2String(npImg)
#    msg = {
#        "type": "proceeded",
#        'detail': 'rawImg',
#        "content": content
#    }
    msg = MSG['PROCEEDED']
    msg['detail'] = 'rawImg'
    msg['content'] = content
    return msg

def detectFace():
    with open(tmpImgPath, 'rb') as f:
        npImg = comm.string2NpImg(f.read())

    if npImg is None:
        return MSG['EMPTY']

    npImg = scaledImg(npImg)
    hasFace = FaceDetector.markAllFacesByDlib(npImg)

    content = comm.npImg2String(npImg)
#    msg = {
#        "type": "proceeded",
#        'detail': 'faceDetected' if hasFace else 'noFace',
#        "content": content
#    }
    msg = MSG['PROCEEDED']
    msg['detail'] = 'faceDetected' if hasFace else 'noFace'
    msg['content'] = content
    return msg

def trainFace(name, identity):
    with open(tmpImgPath, 'rb') as f:
        npImg = comm.string2NpImg(f.read())

    if npImg is None:
        return MSG['EMPTY']

    npImg = scaledImg(npImg)

    if identity not in FaceProcessor.persons:
        FaceProcessor.persons[identity] = name
    elif FaceProcessor.persons[identity] != name:
        FaceProcessor.persons[identity] = name

    align, net = FaceProcessor.align, FaceProcessor.net

    bb = align.getLargestFaceBoundingBox(npImg)
    if bb is None:
        content = comm.npImg2String(npImg)
        msg = MSG['PROCEEDED']
        msg['detail'] = 'noFace'
        msg['content'] = content
        return msg
#        return {
#            "type": "proceeded",
#            'detail': 'noFace',
#            "content": content
#        }

    p1 = (bb.left(), bb.top())
    p2 = (bb.right(), bb.bottom())
    cv2.rectangle(npImg, p1, p2, (0, 255, 0), 2)

    alignedFace = align.align(96, npImg, bb,
        landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
    phash = str(imagehash.phash(Image.fromarray(alignedFace)))
    if phash not in FaceProcessor.images:
        rep = net.forward(alignedFace)
        FaceProcessor.images[phash] = {
            'identity' : identity,
            'rep' : rep
        }

    content = comm.npImg2String(npImg)
    msg = MSG['PROCEEDED']
    msg['detail'] = 'faceTrained'
    msg['content'] = content
    return msg
#    return {
#        "type": "proceeded",
#        'detail': 'faceTrained',
#        "content": content
#    }

def getPersons():
    return FaceProcessor.persons

def saveData():
    FaceProcessor.saveData()

def updateRecognizer():
#    with open(FaceRecognizer.pklpath, 'rb') as f:
#        FaceRecognizer.persons = pickle.load(f)
#    saveData()

    if len(FaceProcessor.persons) <= 1:
        return;
    FaceRecognizer.le = FaceRecognizer.LabelEncoder().fit(
        list(FaceProcessor.persons.keys())
    )
    FaceRecognizer.clf = FaceRecognizer.train()

#recognizerInited = False

def recognizeFace():
#    global recognizerInited
#    if not recognizerInited:
#        updateRecognizer()
#        recognizerInited=True

    le, clf, persons, align, net = \
    FaceRecognizer.le, FaceRecognizer.clf, FaceProcessor.persons, FaceRecognizer.align, FaceRecognizer.net

    if clf is None:
        return MSG['EMPTY']

    with open(tmpImgPath, 'rb') as f:
        npImg = comm.string2NpImg(f.read())

    if npImg is None:
        return MSG['EMPTY']

    npImg = scaledImg(npImg)
    bb = align.getLargestFaceBoundingBox(npImg)
    if bb is None:
        content = comm.npImg2String(npImg)
        msg = MSG['PROCEEDED']
        msg['detail'] = 'noFace'
        msg['content'] = content
        return msg
#        return {
#            "type": "proceeded",
#            'detail': 'noFace',
#            "content": content
#        }

    alignedFace = align.align(96, npImg, bb,
        landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
    rep = net.forward(alignedFace)
    rep1 = rep.reshape(1, -1)
    predictions = clf.predict_proba(rep).ravel()
    maxI = np.argmax(predictions)
    confidence = predictions[maxI]
    identity, name = -1, 'unknown'
    if confidence >= 0.7:
        identity = le.inverse_transform(maxI)
        name = persons[identity]

#        phash = str(imagehash.phash(Image.fromarray(alignedFace)))
#        if phash not in FaceProcessor.images:
#            rep = net.forward(alignedFace)
#            FaceProcessor.images[phash] = {
#                'identity' : identity,
#                'rep' : rep
#            }

    p1 = (bb.left(), bb.top())
    p2 = (bb.right(), bb.bottom())
    cv2.rectangle(npImg, p1, p2, (0, 255, 0), 2)
#    for p in openface.AlignDlib.OUTER_EYES_AND_NOSE:
#        cv2.circle(npImg, center=landmarks[p], radius=3, color=(0, 255, 0), thickness=-1)
    cv2.putText(
        npImg,
        str(identity) + ':' + name,
        (bb.left(), bb.top() - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=0.6,
        color=(0, 255, 0),
        thickness=2
    )

    content = comm.npImg2String(npImg)
#    msg = {
#        "type": "proceeded",
#        'detail': 'faceRecognized',
#        "content": content
#    }
    msg = MSG['PROCEEDED']
    msg['detail'] = 'faceRecognized'
    msg['content'] = content
    return msg

def trainPic(j):
    header = 'data:{};base64,'
    fmts = map(lambda item: 'image/' + item,
        ['jpg', 'jpeg', 'png', 'bmp']
    )

    content = j['content']
    name = j['name']
    identity = int(j['identity'])

    for fmt in fmts:
        imgHeader = header.format(fmt)
        if not content.startswith(imgHeader):
            continue

        deStr = base64.b64decode(content[len(imgHeader):])
        npImg = comm.string2NpImg(deStr)

        npImg = scaledImg(npImg)

        if identity not in FaceProcessor.persons:
            FaceProcessor.persons[identity] = name
        elif FaceProcessor.persons[identity] != name:
            FaceProcessor.persons[identity] = name

        print identity, FaceProcessor.persons[identity]

        align, net = FaceProcessor.align, FaceProcessor.net

        bb = align.getLargestFaceBoundingBox(npImg)
        if bb is None:
            break;

        p1 = (bb.left(), bb.top())
        p2 = (bb.right(), bb.bottom())
        cv2.rectangle(npImg, p1, p2, (0, 255, 0), 2)

        alignedFace = align.align(96, npImg, bb,
            landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
        phash = str(imagehash.phash(Image.fromarray(alignedFace)))
        if phash not in FaceProcessor.images:
            rep = net.forward(alignedFace)
            FaceProcessor.images[phash] = {
                'identity' : identity,
                'rep' : rep
            }

    msg = MSG['UPLOADED_SIZE']
    msg['uploadedSize'] = j['uploadedSize']
    return msg
#    return {
#        'type': 'uploadedSize',
#        'uploadedSize': j['uploadedSize']
#    }
