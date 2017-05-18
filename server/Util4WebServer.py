#!/usr/bin/env python
import cv2, numpy as np, base64, imagehash, pickle
from PIL import Image
import openface
import comm, FaceDetector, FaceProcessor, FaceRecognizer

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
    },
    'DETECTED_FACES': {
        'type': 'detectedFaces',
        'boxes': None
    },
    'RECOGNIZED_FACE': {
        'type': 'recognizedFace',
        'box': None,
        'label': None
    },
    'TRAIN_STOPED': {
        'type': 'trainStoped'
    },
    'PERSON_DELETED': {
        'type': 'personDeleted',
        'identity': None
    }
}

tmpImg = None

def scaledImg(img, scale=0.5):
    sp = img.shape
    h, w = sp[0], sp[1]
    h = int(round(h * scale, 0))
    w = int(round(w * scale, 0))
    return cv2.resize(img, (w, h))

def rawFace():
    # if tmpImg is not None:
    #     with open('tmpImg1', 'wb') as f:
    #         f.write(tmpImg)

    # npImg = comm.bytes2NpImg(tmpImg)
    with open('tmpImg', 'rb') as f:
        npImg = comm.bytes2NpImg(f.read())

    if npImg is None:
        return MSG['EMPTY']

    npImg = scaledImg(npImg)

    content = 'data:image/jpg;base64,' + \
        base64.b64encode(comm.npImg2Bytes(npImg))

    msg = MSG['PROCEEDED']
    msg['detail'] = 'rawImg'
    msg['content'] = content
    return msg

def detectFace():
    with open('tmpImg', 'rb') as f:
        npImg = comm.bytes2NpImg(f.read())

    if npImg is None:
        return MSG['EMPTY']

    npImg = scaledImg(npImg)
    faces = FaceDetector.getAllFaceBoxes(npImg)

    boxes = []

    for face in faces:
        box = {
            'left': face.left(),
            'top': face.top(),
            'right': face.right(),
            'bottom': face.bottom()
        }
        boxes.append(box)

    msg = MSG['DETECTED_FACES']
    msg['boxes'] = boxes
    return msg

def trainFace(name, identity):
    with open('tmpImg', 'rb') as f:
        npImg = comm.bytes2NpImg(f.read())

    if npImg is None:
        return MSG['EMPTY']


    if identity not in FaceProcessor.persons:
        FaceProcessor.persons[identity] = name
    elif FaceProcessor.persons[identity] != name:
        FaceProcessor.persons[identity] = name

    align = FaceProcessor.align
    npImg = scaledImg(npImg)
    bb = align.getLargestFaceBoundingBox(npImg)
    if bb is None:
        return MSG['EMPTY']

    alignedFace = align.align(96, npImg, bb,
        landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
    phash = str(imagehash.phash(Image.fromarray(alignedFace)))
    if phash not in FaceProcessor.images:
        rep = FaceProcessor.net.forward(alignedFace)
        FaceProcessor.images[phash] = {
            'identity' : identity,
            'rep' : rep
        }


    msg = MSG['DETECTED_FACES']
    msg['boxes'] = [
        {
            'left': bb.left(),
            'top': bb.top(),
            'right': bb.right(),
            'bottom': bb.bottom()
        }
    ]
    return msg

def getPersons():
    return FaceProcessor.persons

def initData():
    FaceProcessor.initData()

def saveData():
    FaceProcessor.saveData()

def updateRecognizer():
    if len(FaceProcessor.persons) <= 1:
        return;
    FaceRecognizer.le = FaceRecognizer.LabelEncoder().fit(
        list(FaceProcessor.persons.keys())
    )
    FaceRecognizer.clf = FaceRecognizer.train()

def recognizeFace():
    clf = FaceRecognizer.clf

    if clf is None:
        return MSG['EMPTY']

    with open('tmpImg', 'rb') as f:
        npImg = comm.bytes2NpImg(f.read())

    if npImg is None:
        return MSG['EMPTY']

    align = FaceRecognizer.align
    npImg = scaledImg(npImg)
    bb = align.getLargestFaceBoundingBox(npImg)
    if bb is None:
        return MSG['EMPTY']

    le, persons, net = FaceRecognizer.le, FaceProcessor.persons, FaceRecognizer.net

    alignedFace = align.align(96, npImg, bb,
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

    msg = MSG['RECOGNIZED_FACE']
    msg['box'] = {
        'left': bb.left(),
        'top': bb.top(),
        'right': bb.right(),
        'bottom': bb.bottom()
    }
    msg['label'] = {
        'identity': identity,
        'name': name
    }
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
        npImg = comm.bytes2NpImg(deStr)


        if identity not in FaceProcessor.persons:
            FaceProcessor.persons[identity] = name
        elif FaceProcessor.persons[identity] != name:
            FaceProcessor.persons[identity] = name

        align = FaceProcessor.align
        npImg = scaledImg(npImg)
        bb = align.getLargestFaceBoundingBox(npImg)
        if bb is None:
            break;

        alignedFace = align.align(96, npImg, bb,
            landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
        phash = str(imagehash.phash(Image.fromarray(alignedFace)))
        if phash not in FaceProcessor.images:
            rep = FaceProcessor.net.forward(alignedFace)
            FaceProcessor.images[phash] = {
                'identity' : identity,
                'rep' : rep
            }

    msg = MSG['UPLOADED_SIZE']
    msg['uploadedSize'] = j['uploadedSize']
    return msg

def delPerson(identity):
    persons, images = FaceProcessor.persons, FaceProcessor.images
    if identity in persons:
        persons.pop(identity)

    for key in images.keys():
        if images[key]['identity'] == identity:
            images.pop(key)