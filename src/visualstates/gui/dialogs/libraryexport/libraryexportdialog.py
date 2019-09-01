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


from github import Github
import sys
from PyQt5.QtWidgets import QDialog, QLabel, QLineEdit, \
    QPushButton, QApplication, QHBoxLayout, QVBoxLayout, \
    QMessageBox, QPlainTextEdit
from PyQt5.QtCore import *
from ....githubtools.uploadfile import UploadFile

class LibraryExportDialog(QDialog):
    def __init__(self, username, password, xmlFile):
        super(QDialog, self).__init__()
        self.setWindowTitle("Export to Online Library")
        self.username = username
        self.password = password
        self.xmlFile = xmlFile
        self.UI = []

        self.drawDescWindow()

    def drawDescWindow(self):
        self.setFixedSize(500, 350)
        VLayout = QVBoxLayout()
        VLayout.setAlignment(Qt.AlignTop)

        titleLblStyleSheet = 'QLabel {font-weight: bold;}'
        rowLayout = QHBoxLayout()
        titleLbl = QLabel('Enter name and a short description for the automata behaviour')
        titleLbl.setStyleSheet(titleLblStyleSheet)
        rowLayout.addWidget(titleLbl)
        VLayout.addLayout(rowLayout)

        rowLayout = QHBoxLayout()
        nameLbl = QLabel('Name:')
        nameLbl.setFixedWidth(50)
        rowLayout.addWidget(nameLbl)
        self.nameBox = QLineEdit()
        rowLayout.addWidget(self.nameBox)
        VLayout.addLayout(rowLayout)
        helpLbl = QLabel('Names can only contain numbers, lowercase alphabets and spaces')
        helpLbl.setStyleSheet('QLabel{font:italic;}')
        VLayout.addWidget(helpLbl)

        VLayout.addWidget(QLabel(''))

        VLayout.addWidget(QLabel('Description:'))
        self.descBox = QPlainTextEdit()
        self.descBox.setMinimumHeight(100)
        VLayout.addWidget(self.descBox)

        btnLayout = QHBoxLayout()
        btnLayout.setAlignment(Qt.AlignRight)
        self.submitBtn = QPushButton('Submit')
        self.submitBtn.setFixedWidth(80)
        self.submitBtn.clicked.connect(self.submitClicked)
        btnLayout.addWidget(self.submitBtn)
        VLayout.addLayout(btnLayout)
        

        rowLayout = QHBoxLayout()
        self.statusLbl = QLabel('')
        rowLayout.addWidget(self.statusLbl)
        VLayout.addLayout(rowLayout)
        self.setLayout(VLayout)

    def submitClicked(self):
        self.submitBtn.setEnabled(False)
        self.submitBtn.repaint()
        self.setStatus("Checking Name . . .")
        self.name = self.nameBox.text().strip()
        self.description = self.descBox.toPlainText()
        if self.description == "":
            self.description = "No description provided"
        if self.nameCheck():
            self.setStatus("Uploading File . . .")
            self.uploadFile = UploadFile(self.username, self.password, self.name, self.description, self.xmlFile)
            self.uploadFile.finished.connect(self.accept)
            self.uploadFile.error.connect(self.errorInUpload)
            self.uploadFile.status.connect(self.setStatus)
            self.uploadFile.start()
        else:
            self.submitBtn.setEnabled(True)
            self.setStatus("")

    def errorInUpload(self, errorStr):
        QMessageBox.warning(self, "Error", errorStr)
        self.submitBtn.setEnabled(True)
        self.setStatus("")

    def setStatus(self, text):
        self.statusLbl.setText(text)
        self.statusLbl.repaint()

    def nameCheck(self):
        try:
            g = Github("", "")
            repo = g.get_repo('sudo-panda/automata-library')
            files = repo.get_contents("")
            for file in files:
                if file.path == self.name:
                    return False
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))

        if not self.name.replace(" ","").isalnum():
            QMessageBox.warning(self, "Name Error", "Names cannot contain special characters except spaces")
            return False
        if not self.name.islower():
            QMessageBox.warning(self, "Name Error", "Names should be lower case")
            return False
        return True


if __name__ == '__main__':
    app = QApplication(sys.argv)

    dialog = LibraryExportDialog("", "", '')
    if dialog.exec_():
        name = dialog.name
        description = dialog.description