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

from PyQt5.QtCore import QThread, pyqtSignal
from github import Github

class DownloadFile(QThread):
    fileStr = pyqtSignal('QString')
    def __init__(self, fileName, repoName="sudo-panda/automata-library"):
        super(QThread, self).__init__()
        self.fileName = fileName
        self.repoName = repoName

    def run(self):
        try:
            g = Github("", "")
            repo = g.get_repo(self.repoName)  # TODO: Replace with actual repo
            catalogue = repo.get_contents(self.fileName)
            self.fileStr.emit(catalogue.decoded_content)
        except:
            self.fileStr.emit("")