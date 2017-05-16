#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import numpy as np, cv2, os
#import argparse
#import pickle, comm
#f = open('/home/anchore/faces.pkl', 'rb')
#ps = pickle.load(f)
##ps.pop('1')

#d=pickle.load(f)

#f.close()

import os
summ = 0

def getLineSum(filePath):
	with open(filePath, 'r') as f:
		return len(f.readlines())

def walkPath(basePath):
	global summ
	arr = [basePath +'/' + i for i in os.listdir(basePath)]
	# print(arr)
	for i in arr:
		if os.path.isdir(i):
			if i.endswith('vendor'):
				continue
			walkPath(i)
		elif os.path.isfile(i):
			if not i.endswith('.py'):
				continue
			summ += getLineSum(i)

walkPath('../server')
print(summ)