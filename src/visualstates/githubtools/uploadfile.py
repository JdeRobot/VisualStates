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

import re
from github import Github
from xml.dom import minidom
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QMessageBox, QDialog
from requests.exceptions import ConnectionError, ReadTimeout

class UploadFile(QThread):
    finished = pyqtSignal()
    error = pyqtSignal(str)
    status = pyqtSignal(str)

    def __init__(self, username, password, name, description, xmlFile):
        super(QThread, self).__init__()
        self.username = username
        self.password = password
        self.name = name
        self.description = description
        self.xmlFile = xmlFile

    def run(self):
        try:
            self.upload()
        except ConnectionError or ReadTimeout:
            self.error.emit("Cannot connect to Github")
            self.close()
        except Exception as e:
            self.error.emit(str(e))

    def upload(self):
        branchName = (self.name.lower()).replace(' ', '-')
        filename = (self.name).replace(' ', '_')
        g = Github(self.username, self.password)

        upUser = g.get_user('sudo-panda')  # TODO:Change the repo to the actual one
        upRepo = upUser.get_repo("automata-library")  # TODO:Change the repo to the actual one

        self.status.emit("Forking library . . .")
        forkUser = g.get_user()
        forkRepo = forkUser.create_fork(upRepo)
        pulls = forkRepo.get_pulls(state='open', sort='created', base='master', head=upUser.login + ":master")
        count = 0
        for pr in pulls:
            count += 1
            pull = forkRepo.get_pull(pr.number)
            pull.merge()
        if count == 0:
            # TODO: Find better way to do this
            try:
                fork_pullrequest = forkRepo.create_pull("Merge upstream master into master", "", '{}'.format('master'),
                                                        '{}:{}'.format(upUser.login, 'master'), False)
                fork_pullrequest.merge()
            except Exception as e:
                pass

        self.status.emit("Checking for existing behaviours . . .")
        branches = list(forkRepo.get_branches())
        activeBranch = None
        exists = False
        for branch in branches:
            if branch.name == branchName:
                activeBranch = forkRepo.get_branch(branch=branchName)
                exists = True
        if activeBranch is None:
            masterBranch = forkRepo.get_branch(branch='master')
            forkRepo.create_git_ref(ref='refs/heads/' + branchName, sha=masterBranch.commit.sha)
            activeBranch = forkRepo.get_branch(branch=branchName)

        if exists:
            self.status.emit("Updating existing behaviour . . .")
            catalogue = forkRepo.get_contents("Catalogue.xml", ref=branchName)
            doc = minidom.parseString(catalogue.decoded_content)
            behaviour = forkRepo.get_contents(self.name + "/" + filename + ".xml", ref=branchName)
            forkRepo.update_file("Catalogue.xml", "Edit behaviour description in catalogue",
                                 changeCatalogue(doc, self.name, self.description), catalogue.sha, branch=branchName)
            forkRepo.update_file(self.name + "/" + filename + ".xml", "Edit behaviour file",
                                 self.xmlFile, behaviour.sha, branch=branchName)

            pulls = upRepo.get_pulls(state='open', sort='created', base='master', head=forkUser.login + ":" + branchName)
            pull_exists = False
            for pr in pulls:
                pr.edit(title="Add " + self.name + " behaviour", body=self.description)
                pull_exists = True
            if not pull_exists:
                self.setStatus("Creating pull request")
                upRepo.create_pull("Add " + self.name + " behaviour", self.description,
                                                 '{}'.format('master'), '{}:{}'.format(forkUser.login, branchName), True)
        else:
            self.status.emit("Creating new behaviour . . .")
            catalogue = forkRepo.get_contents("Catalogue.xml", ref=branchName)

            # TODO: Remove the try except after first commit to library
            try:
                doc = minidom.parseString(catalogue.decoded_content)
            except:
                doc = minidom.Document()
                catElement = doc.createElement('Catalogue')
                doc.appendChild(catElement)

            forkRepo.update_file("Catalogue.xml", "Add behaviour to catalogue",
                                 changeCatalogue(doc, self.name, self.description), catalogue.sha, branch=branchName)
            forkRepo.create_file(self.name + "/" + filename + ".xml", "Add behaviour file",
                                 self.xmlFile, branch=branchName)

            self.status.emit("Creating pull request")
            upRepo.create_pull("Add " + self.name + " behaviour", self.description,
                                             '{}'.format('master'), '{}:{}'.format(forkUser.login, branchName), True)
        self.finished.emit()

def changeCatalogue(doc, name, description):
    exists = False
    behaviourList = doc.getElementsByTagName('Catalogue')[0].getElementsByTagName('behaviour')
    for behElement in behaviourList:
        if behElement.getAttribute('name') == name:
            descElement = behElement.getElementsByTagName('description')[0]
            descElement.childNodes[0].nodeValue = description
            exists = True
    if not exists:
        behElement = doc.createElement('behaviour')
        behElement.setAttribute('name', name)
        descElement = doc.createElement('description')
        descElement.appendChild(doc.createTextNode(description))
        behElement.appendChild(descElement)
        doc.getElementsByTagName('Catalogue')[0].appendChild(behElement)
    xmlStr = re.sub(r'(\n( *))(\1)+', r'\1', doc.toprettyxml(indent='  '))
    xmlStr = re.sub(r'\n( *)\n', '\n', xmlStr)
    return xmlStr