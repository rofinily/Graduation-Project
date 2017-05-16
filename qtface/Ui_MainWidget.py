# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MainWidget.ui'
#
# Created: Thu Apr 27 14:48:12 2017
#      by: pyside-uic 0.2.15 running on PySide 1.2.4
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_MainWidget(object):
    def setupUi(self, MainWidget):
        MainWidget.setObjectName("MainWidget")
        MainWidget.resize(572, 298)
        self.horizontalLayoutWidget = QtGui.QWidget(MainWidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(30, 20, 512, 242))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout_4 = QtGui.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.faceImgLabel = QtGui.QLabel(self.horizontalLayoutWidget)
        self.faceImgLabel.setMinimumSize(QtCore.QSize(320, 240))
        self.faceImgLabel.setObjectName("faceImgLabel")
        self.horizontalLayout_4.addWidget(self.faceImgLabel)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.detectButton = QtGui.QPushButton(self.horizontalLayoutWidget)
        self.detectButton.setObjectName("detectButton")
        self.verticalLayout.addWidget(self.detectButton)
        self.trainCheckBox = QtGui.QCheckBox(self.horizontalLayoutWidget)
        self.trainCheckBox.setObjectName("trainCheckBox")
        self.verticalLayout.addWidget(self.trainCheckBox)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtGui.QLabel(self.horizontalLayoutWidget)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.nameLineEdit = QtGui.QLineEdit(self.horizontalLayoutWidget)
        self.nameLineEdit.setObjectName("nameLineEdit")
        self.horizontalLayout.addWidget(self.nameLineEdit)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_3 = QtGui.QLabel(self.horizontalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_2.addWidget(self.label_3)
        self.idLineEdit = QtGui.QLineEdit(self.horizontalLayoutWidget)
        self.idLineEdit.setObjectName("idLineEdit")
        self.horizontalLayout_2.addWidget(self.idLineEdit)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.msgLabel = QtGui.QLabel(self.horizontalLayoutWidget)
        self.msgLabel.setStyleSheet("color: rgb(255, 0, 0);")
        self.msgLabel.setText("")
        self.msgLabel.setObjectName("msgLabel")
        self.verticalLayout.addWidget(self.msgLabel)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_2 = QtGui.QLabel(self.horizontalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_3.addWidget(self.label_2)
        self.personsComboBox = QtGui.QComboBox(self.horizontalLayoutWidget)
        self.personsComboBox.setObjectName("personsComboBox")
        self.horizontalLayout_3.addWidget(self.personsComboBox)
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.recognizeButton = QtGui.QPushButton(self.horizontalLayoutWidget)
        self.recognizeButton.setObjectName("recognizeButton")
        self.verticalLayout.addWidget(self.recognizeButton)
        spacerItem1 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem1)
        self.horizontalLayout_4.addLayout(self.verticalLayout)

        self.retranslateUi(MainWidget)
        QtCore.QMetaObject.connectSlotsByName(MainWidget)

    def retranslateUi(self, MainWidget):
        MainWidget.setWindowTitle(QtGui.QApplication.translate("MainWidget", "Main", None, QtGui.QApplication.UnicodeUTF8))
        self.detectButton.setText(QtGui.QApplication.translate("MainWidget", "Face Detect", None, QtGui.QApplication.UnicodeUTF8))
        self.trainCheckBox.setText(QtGui.QApplication.translate("MainWidget", "Face Train", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("MainWidget", "   Name", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("MainWidget", "          ID", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("MainWidget", "Trained", None, QtGui.QApplication.UnicodeUTF8))
        self.recognizeButton.setText(QtGui.QApplication.translate("MainWidget", "Face Recognize", None, QtGui.QApplication.UnicodeUTF8))

