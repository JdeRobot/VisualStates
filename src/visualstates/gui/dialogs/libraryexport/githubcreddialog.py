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
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox
from PyQt5.QtCore import Qt
from github.GithubException import BadCredentialsException
from requests.exceptions import ConnectionError


class GithubCredentialsDialog(QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        self.setWindowTitle("Export to Online Library")

        self.drawCredWindow()

    def drawCredWindow(self):
        self.setFixedSize(300, 150)
        VLayout = QVBoxLayout()
        VLayout.setAlignment(Qt.AlignTop)

        titleLblStyleSheet = 'QLabel {font-weight: bold;}'
        rowLayout = QHBoxLayout()
        titleLbl = QLabel('Enter Github username and password')
        titleLbl.setStyleSheet(titleLblStyleSheet)
        rowLayout.addWidget(titleLbl)
        VLayout.addLayout(rowLayout)

        rowLayout = QHBoxLayout()
        usernameLbl = QLabel('Username:')
        usernameLbl.setFixedWidth(80)
        rowLayout.addWidget(usernameLbl)
        self.usernameBox = QLineEdit()
        self.usernameBox.setFixedWidth(190)
        rowLayout.addWidget(self.usernameBox)
        VLayout.addLayout(rowLayout)

        rowLayout = QHBoxLayout()
        passwordLbl = QLabel('Password:')
        passwordLbl.setFixedWidth(80)
        rowLayout.addWidget(passwordLbl)
        self.passwordBox = QLineEdit()
        self.passwordBox.setFixedWidth(190)
        self.passwordBox.setEchoMode(QLineEdit.Password)
        rowLayout.addWidget(self.passwordBox)
        VLayout.addLayout(rowLayout)

        btnLayout = QHBoxLayout()
        btnLayout.setAlignment(Qt.AlignRight)
        self.submitBtn = QPushButton('Submit')
        self.submitBtn.setFixedWidth(100)
        self.submitBtn.clicked.connect(self.submitClicked)
        btnLayout.addWidget(self.submitBtn)
        VLayout.addLayout(btnLayout)
        self.setLayout(VLayout)

    def submitClicked(self):
        self.submitBtn.setEnabled(False)
        self.submitBtn.setText('Checking . . .')
        self.submitBtn.repaint()
        if self.usernameBox.text() == '':
            QMessageBox.warning(self, 'Error', 'Username field is empty')
        elif self.passwordBox.text() == '':
            QMessageBox.warning(self, 'Error', 'Password field is empty')
        else:
            try:
                name = Github(self.usernameBox.text(), self.passwordBox.text()).get_user().name
                self.username = self.usernameBox.text()
                self.password = self.passwordBox.text()
                self.accept()
            except BadCredentialsException:
                QMessageBox.warning(self, 'Error', 'Incorrect username or password')
            except ConnectionError:
                QMessageBox.warning(self, "Error", "Cannot connect to Github")
                self.close()
            except Exception as e:
                print e
        self.submitBtn.setEnabled(True)
        self.submitBtn.setText('Submit')
