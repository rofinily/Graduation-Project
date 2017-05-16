#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, time, thread, re, atexit
from PySide.QtCore import QByteArray, QObject, Signal, Slot, Qt
from PySide.QtGui import QWidget, QPixmap, QApplication
from Ui_MainWidget import Ui_MainWidget
import Util4Client as u4c

lock = thread.allocate_lock()

MSG = {
    'NOTHING': {
        'cmd': 'nothing',
        'interval': 1000
    },
    'RAW': {
        'cmd': 'raw',
        'interval': 250
    },
    'DETECT': {
        'cmd': 'detect',
        'interval': 300
    },
    'TRAIN': {
        'cmd': 'train',
        'name': None,
        'identity': None,
        'interval': 750
    },
    'STOP_TRAIN': {
        'cmd': 'stopTrain',
        'interval': 750
    },
    'RECOGNIZE': {
        'cmd': 'recognize',
        'interval': 750
    },
    'GET_PERSONS': {
        'cmd': 'getPersons',
        'interval': 500
    }
}

class MainWidget(QWidget):

    imgSig = Signal(QByteArray)

    def __init__(self):
        super(MainWidget, self).__init__()
        self.ui = Ui_MainWidget()
        self.ui.setupUi(self)

        self.ui.detectButton.clicked.connect(self.detectSlot)
        self.ui.trainCheckBox.clicked.connect(self.trainSlot)
        self.ui.recognizeButton.clicked.connect(self.recognizeSlot)
        self.ui.personsComboBox.currentIndexChanged.connect(self.personChangeSlot)
        self.imgSig.connect(self.imgSigSlot)

        self.msg = MSG['RAW']
#        self.lock = thread.allocate_lock()

        u4c.updateRecognizer()

        self.fitComboBox()

    def detectSlot(self):
        self.msg = MSG['DETECT']

    def trainSlot(self):
        trainFlag = True if self.ui.trainCheckBox.checkState() is Qt.Checked else False


        if trainFlag:
            name = self.ui.nameLineEdit.text()
            identity = self.ui.idLineEdit.text()

            if self.validateNameId(name, identity) is False:
                self.ui.trainCheckBox.setCheckState(Qt.Unchecked)
                self.ui.msgLabel.setText('Wrong Format: name or ID')
                return

            self.ui.msgLabel.setText('')
            self.ui.detectButton.setEnabled(False)
            self.ui.nameLineEdit.setEnabled(False)
            self.ui.idLineEdit.setEnabled(False)
            self.ui.recognizeButton.setEnabled(False)

            self.msg = MSG['TRAIN']
            self.msg['name'] = name
            self.msg['identity'] = identity
        else:
            self.msg = MSG['RAW']

            lock.acquire()
            u4c.saveData()
            lock.release()

            u4c.updateRecognizer()

            self.fitComboBox()

            self.ui.detectButton.setEnabled(True)
            self.ui.nameLineEdit.setEnabled(True)
            self.ui.idLineEdit.setEnabled(True)
            self.ui.recognizeButton.setEnabled(True)

    def recognizeSlot(self):
        self.msg = MSG['RECOGNIZE']

    @Slot(int)
    def personChangeSlot(self, idx):
        if self.ui.trainCheckBox.checkState() is not Qt.Checked:
            p = self.ui.personsComboBox.itemText(idx)
            print(p.split(':'))
            identity, name = p.split(':')
            self.ui.nameLineEdit.setText(name)
            self.ui.idLineEdit.setText(identity)

    def validateNameId(self, name, identity):
        if name == '' or identity == '':
            return False
        if re.match(r'^\d+$', identity) is None:
            return False
        return True

    def imgSigSlot(self, qByteArray):
        pixmap = QPixmap()
        pixmap.loadFromData(qByteArray)
        self.ui.faceImgLabel.setPixmap(pixmap)

    def emitData(self):
        while True:
            if self.msg['cmd'] == 'raw':
                respMsg = u4c.rawFace()
                self.handleMsg(respMsg)
            elif self.msg['cmd'] == 'detect':
                respMsg = u4c.detectFace()
                self.handleMsg(respMsg)
            elif self.msg['cmd'] == 'train':
                respMsg = u4c.trainFace(self.msg['name'], int(self.msg['identity']))
                self.handleMsg(respMsg)
            elif self.msg['cmd'] == 'recognize':

                lock.acquire()
                respMsg = u4c.recognizeFace()
                lock.release()

                self.handleMsg(respMsg)
#            elif self.msg['cmd'] == 'getPersons':
#                respMsg = u4c.getPersons()
#                self.handleMsg(respMsg)
            else:
                pass

            time.sleep(self.msg['interval'] / 1000)

    def handleMsg(self, respMsg):
        if respMsg['type'] == 'proceeded':
            self.imgSig.emit(QByteArray.fromRawData(respMsg['content']))
#        elif respMsg['type'] == 'persons':
#            pass
        else:
            pass

    def fitComboBox(self):
        self.ui.personsComboBox.clear()
        persons = u4c.getPersons()
        for k in persons:
            self.ui.personsComboBox.addItem('{}:{}'.format(k, persons[k]))

@atexit.register
def onExit():
    lock.acquire()
    u4c.saveData()
    lock.release()

if __name__ == '__main__':
    app = QApplication(sys.argv)

    mw = MainWidget()
    thread.start_new_thread(mw.emitData, ())
    mw.show()
    sys.exit(app.exec_())
