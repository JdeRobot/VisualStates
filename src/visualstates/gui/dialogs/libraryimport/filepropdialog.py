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

from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPlainTextEdit, QPushButton
from PyQt5.QtCore import Qt
from ....githubtools.downloadfile import DownloadFile

class FilePropertiesDialog(QDialog):
    def __init__(self, name, description):
        super(QDialog, self).__init__()
        self.setMinimumSize(600, 500)
        self.name = name
        self.description = description
        self.fileStr = ''
        self.setWindowTitle(self.name)
        self.drawWindow()

    def drawWindow(self):
        VLayout = QVBoxLayout()
        titleLblStyleSheet = 'QLabel {font-weight: bold;}'

        rowLayout = QHBoxLayout()
        titleLbl = QLabel('Description:')
        titleLbl.setMinimumWidth(200)
        titleLbl.setStyleSheet(titleLblStyleSheet)
        rowLayout.addWidget(titleLbl)
        # titleLbl = QLabel('Snapshot:')
        # titleLbl.setMinimumWidth(200)
        # titleLbl.setStyleSheet(titleLblStyleSheet)
        # rowLayout.addWidget(titleLbl)
        VLayout.addLayout(rowLayout)

        rowLayout = QHBoxLayout()
        descBox = QPlainTextEdit(self.description)
        descBox.setReadOnly(True)
        rowLayout.addWidget(descBox)
        # descBox = QPlainTextEdit('')
        # rowLayout.addWidget(descBox)
        VLayout.addLayout(rowLayout)

        btnLayout = QHBoxLayout()
        btnLayout.setAlignment(Qt.AlignRight)
        self.importBtn = QPushButton("Import")
        self.importBtn.setFixedWidth(80)
        self.importBtn.clicked.connect(self.importClicked)
        btnLayout.addWidget(self.importBtn)
        self.cancelBtn = QPushButton("Cancel")
        self.cancelBtn.setFixedWidth(80)
        self.cancelBtn.clicked.connect(self.cancelClicked)
        btnLayout.addWidget(self.cancelBtn)
        VLayout.addLayout(btnLayout)
        self.statusLbl = QLabel('')
        VLayout.addWidget(self.statusLbl)
        self.setLayout(VLayout)


    def cancelClicked(self):
        self.close()

    def setFileStr(self, fileStr):
        if fileStr == "":
            self.setStatus("Error occurred in downloading File")
            self.cancelBtn.setEnabled(True)
            self.importBtn.setEnabled(True)
            return
        self.fileStr = fileStr
        self.setStatus("File Downloaded")
        self.accept()

    def setStatus(self, text):
        self.statusLbl.setText(text)
        self.statusLbl.repaint()

    def importClicked(self):
        self.setStatus("Downloading file . . .")

        self.cancelBtn.setEnabled(False)
        self.importBtn.setEnabled(False)

        self.downloadFile = DownloadFile(self.name + "/" + self.name.replace(' ', '_') + ".xml")
        self.downloadFile.fileStr.connect(self.setFileStr)
        self.downloadFile.start()