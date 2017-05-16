#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
curDir = os.getcwd()
uiFiles = filter(lambda item: item.endswith('.ui'), os.listdir(curDir))
for i in uiFiles:
#    pyName = 'Ui_{}.py'.format(i[0].lower() + i[1:-3])
    os.system('pyside-uic {} -o Ui_{}.py'.format(i, i[:-3]))
