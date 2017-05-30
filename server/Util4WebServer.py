#!/usr/bin/env python
import base64
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

header = 'data:{};base64,'
fmts = map(lambda item: 'image/' + item,
    ['jpg', 'jpeg', 'png', 'bmp']
)

def getTmpImg():
    with open('tmpImg', 'rb') as f:
        npImg = comm.bytes2NpImg(f.read())
    
    return npImg

def rawFace():
    npImg = getTmpImg()

    if npImg is None:
        return MSG['EMPTY']

    npImg = comm.scaledImg(npImg)

    content = 'data:image/jpg;base64,' + \
        base64.b64encode(comm.npImg2Bytes(npImg))

    msg = MSG['PROCEEDED']
    msg['detail'] = 'rawImg'
    msg['content'] = content
    return msg

def detectFace():
    npImg = getTmpImg()

    if npImg is None:
        return MSG['EMPTY']

    npImg = comm.scaledImg(npImg)
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
    npImg = getTmpImg()

    box = FaceProcessor.train(identity, name, npImg)

    if box is None:
        return MSG['EMPTY']

    msg = MSG['DETECTED_FACES']
    msg['boxes'] = [
        {
            'left': box.left(),
            'top': box.top(),
            'right': box.right(),
            'bottom': box.bottom()
        }
    ]
    return msg

def getPersons():
    return FaceProcessor.getPersons()

def initData():
    FaceProcessor.initData()

def saveData():
    FaceProcessor.saveData()

def updateRecognizer():
    FaceRecognizer.updateRecognizer()

def recognizeFace():
    npImg = getTmpImg()
    
    msg = FaceRecognizer.recognize(npImg)

    if msg is None:
        return MSG['EMPTY']

    identity = msg['identity']
    name = msg['name']
    confidence = msg['confidence']
    box = msg['box']

    print(name, confidence)

    if confidence < 0.7:
        identity, name = -1, 'unknown'

    msg = MSG['RECOGNIZED_FACE']
    msg['box'] = {
        'left': box.left(),
        'top': box.top(),
        'right': box.right(),
        'bottom': box.bottom()
    }
    msg['label'] = {
        'identity': identity,
        'name': name
    }
    return msg

def trainPic(j):
    content = j['content']
    name = j['name']
    identity = int(j['identity'])

    for fmt in fmts:
        imgHeader = header.format(fmt)
        if not content.startswith(imgHeader):
            continue

        deBytes = base64.b64decode(content[len(imgHeader):])
        npImg = comm.bytes2NpImg(deBytes)

        FaceProcessor.train(identity, name, npImg)

        break

    msg = MSG['UPLOADED_SIZE']
    msg['uploadedSize'] = j['uploadedSize']
    return msg

def delPerson(identity):
    FaceProcessor.delPerson(identity)