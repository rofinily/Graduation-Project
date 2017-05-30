#!/usr/bin/env python
import numpy as np, cv2
import openface

FPS = 20
IMG_SIZE = 921600 * 2

align = openface.AlignDlib(
	'/home/anchore/openface/models/dlib/shape_predictor_68_face_landmarks.dat')
net = openface.TorchNeuralNet(
	model='/home/anchore/openface/models/openface/nn4.small2.v1.t7',
	imgDim=96,
	cuda=False
)

def bytes2NpImg(byteArray):
	if byteArray is None:
		return None
	npArr = np.fromstring(byteArray, np.uint8)
	npImg = cv2.imdecode(npArr, cv2.CV_LOAD_IMAGE_COLOR)
	return npImg

def npImg2Bytes(npImg):
	return np.array(cv2.imencode('.jpg', npImg)[1]).tostring()

def scaledImg(img, scale=0.5):
    sp = img.shape
    h, w = sp[0], sp[1]
    h = int(round(h * scale, 0))
    w = int(round(w * scale, 0))
    return cv2.resize(img, (w, h))

if __name__ == '__main__':
	print 'comm'
