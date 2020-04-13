'''
   Copyright (C) 1997-2019 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Baidyanath Kundu (kundubaidya99@gmail.com)

  '''

from xml.dom import minidom
import sys
from PyQt5.QtWidgets import QDialog, QLabel, QPushButton,\
    QApplication, QHBoxLayout, QVBoxLayout, QScrollArea, \
    QGroupBox, QBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal
from .filepropdialog import FilePropertiesDialog
from ...tools.elidedlabel import ElidedLabel
from ....githubtools.downloadfile import DownloadFile

class FileImportDialog(QDialog):
    fileStr = pyqtSignal('QString')
    def __init__(self):
        super(QDialog, self).__init__()
        self.setWindowTitle("Import behaviour from online library")
        self.setMinimumSize(700, 450)
        self.drawWindow()

    def drawWindow(self):
        VLayout = QVBoxLayout()

        rowLayout = QHBoxLayout()

        titleLblStyleSheet = 'QLabel {font-weight: bold;}'
        rowLayout.setAlignment(Qt.AlignLeft)
        rowLayout.addSpacing(10)
        titleLbl = QLabel('Select the behaviour to import:')
        titleLbl.setStyleSheet(titleLblStyleSheet)
        rowLayout.addWidget(titleLbl)

        VLayout.addLayout(rowLayout)

        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scrollVlayout = QVBoxLayout()
        self.scrollVlayout.setDirection(QBoxLayout.TopToBottom)
        self.scrollVlayout.setAlignment(Qt.AlignTop)
        dummyBox = QGroupBox()
        dummyBox.setStyleSheet('QGroupBox {padding: 0px; margin: 0px;}')
        dummyBox.setLayout(self.scrollVlayout)
        scrollArea.setWidget(dummyBox)
        VLayout.addWidget(scrollArea)

        btnLayout = QHBoxLayout()
        btnLayout.setAlignment(Qt.AlignRight)
        cancelBtn = QPushButton("Cancel")
        cancelBtn.setFixedWidth(80)
        cancelBtn.clicked.connect(self.cancelClicked)
        btnLayout.addWidget(cancelBtn)
        VLayout.addLayout(btnLayout)

        self.statusLbl = QLabel('')
        VLayout.addWidget(self.statusLbl)


        self.setLayout(VLayout)
        self.show()

        self.setStatus("Fetching Catalogue . . .")

        self.getCatalogue = DownloadFile("Catalogue.xml")
        self.getCatalogue.fileStr.connect(self.displayCatalogue)
        self.getCatalogue.start()

    def displayCatalogue(self, catalogue):
        if catalogue == "":
            self.setStatus("Error occurred in fetching Catalogue")
            return
        self.setStatus("Displaying Catalogue . . .")
        self.doc = minidom.parseString(catalogue)
        behaviourList = self.doc.getElementsByTagName('Catalogue')[0].getElementsByTagName('behaviour')
        count = 0
        for behElement in behaviourList:
            name = behElement.getAttribute('name')
            description = behElement.getElementsByTagName('description')[0].childNodes[0].nodeValue
            self.addBehaviour(count, name, description)
            count += 1
        self.setStatus("")

    def addBehaviour(self, id, name, description):
        nameLblStyleSheet = 'QLabel {font-weight: bold;}'
        rowLayout = QHBoxLayout()

        behLayout = QVBoxLayout()
        nameLbl = ElidedLabel(name)
        nameLbl.setMinimumWidth(530)
        nameLbl.setToolTip(name)
        nameLbl.setStyleSheet(nameLblStyleSheet)

        behLayout.addWidget(nameLbl)
        descLbl = ElidedLabel(description)
        descLbl.setMinimumWidth(530)
        descLbl.setFixedHeight(17)
        descLbl.setAlignment(Qt.AlignTop)
        behLayout.addWidget(descLbl)

        rowLayout.addLayout(behLayout)

        selectBtn = QPushButton('Select')
        selectBtn.setObjectName(str(id))
        selectBtn.setFixedWidth(100)
        selectBtn.clicked.connect(self.selected)
        rowLayout.addWidget(selectBtn)
        self.scrollVlayout.addLayout(rowLayout)

    def selected(self):
        id = int(self.sender().objectName())
        behElement = self.doc.getElementsByTagName('Catalogue')[0].getElementsByTagName('behaviour')[id]
        name = behElement.getAttribute('name')
        description = behElement.getElementsByTagName('description')[0].childNodes[0].nodeValue
        behDialog = FilePropertiesDialog(name, description)
        if behDialog.exec_():
            self.fileStr.emit(behDialog.fileStr)
            self.accept()

    def setStatus(self, text):
        self.statusLbl.setText(text)
        self.statusLbl.repaint()

    def cancelClicked(self):
        self.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = FileImportDialog()
    dialog.exec_()