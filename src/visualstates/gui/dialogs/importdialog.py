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
import sys
from PyQt5.QtWidgets import QDialog, QLabel,  \
    QPushButton, QApplication, QHBoxLayout, QVBoxLayout, \
    QScrollArea, QGroupBox, QBoxLayout, QCheckBox
from PyQt5.QtCore import pyqtSignal, Qt
from visualstates.gui.dialogs.paramprop import ParamPropDialog
from ..tools.elidedlabel import ElidedLabel
from ..tools.clickablelabel import ClickableLabel
from functools import partial

class ImportDialog(QDialog):
    paramsChanged = pyqtSignal(list)

    def __init__(self, name, file):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.file = file
        self.checkBoxList = []
        self.setMinimumSize(800, 500)

        self.drawWindow()

    def drawWindow(self):
        VLayout = QVBoxLayout()

        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        #scrollArea.setStyleSheet('QScrollArea {border: 0px;}')
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
        doneBtn = QPushButton("Done")
        doneBtn.setFixedWidth(80)
        doneBtn.clicked.connect(self.doneClicked)
        btnLayout.addWidget(doneBtn)
        VLayout.addLayout(btnLayout)
        self.addStates(self.scrollVlayout, self.file[0], None, root=True)
        self.setLayout(VLayout)

    def addStates(self, layout, state, parentStateUIs, root=False):
        titleLblStyleSheet = 'QLabel:enabled{font-weight:bold; color:black;} ' \
                             'QLabel:disabled{font-weight:bold; color:grey;}'
        childTitleLblStyleSheet = 'QLabel {font:italic; font-weight:bold;}'
        bulletLblStyleSheet = 'QLabel {font-size: 25px; font-weight:bold;}'
        stateUIs = []
        if not root:
            rowLayout = QHBoxLayout()
            rowLayout.setAlignment(Qt.AlignLeft)
            rowLayout.addSpacing(5)
            checkBox = QCheckBox()
            checkBox.setFixedWidth(20)
            checkBox.setStyleSheet(bulletLblStyleSheet)
            checkBox.setChecked(True)
            checkBox.stateChanged.connect(partial(self.setStateEnabled, stateUIs))
            if len(self.checkBoxList) < state.id:
                self.checkBoxList.extend([None] * (state.id - len(self.checkBoxList)))
            self.checkBoxList[state.id - 1] = checkBox
            if parentStateUIs is not None:
                parentStateUIs.append(checkBox)
            rowLayout.addWidget(checkBox)
            nameLbl = QLabel(state.getName())
            nameLbl.setStyleSheet(titleLblStyleSheet)
            rowLayout.addWidget(nameLbl)
            stateUIs.append(nameLbl)
            if len(state.getNamespace().getParams()) > 0 or len(state.getChildren()) > 0:
                viewLbl = ClickableLabel(u'\u25BE')
                viewLbl.setMinimumWidth(20)
                rowLayout.addWidget(viewLbl)
            layout.addLayout(rowLayout)

            if len(state.getNamespace().getParams()) > 0 or len(state.getChildren()) > 0:
                rowLayout = QHBoxLayout()
                rowLayout.addSpacing(14)
                stateLayout = QVBoxLayout()
                stateLayout.setDirection(QBoxLayout.TopToBottom)
                stateLayout.setAlignment(Qt.AlignTop)
                dummyBox = QGroupBox()
                dummyBox.setStyleSheet('QGroupBox {padding: 0px; margin: 0px; border-left: 1px solid gray; '
                                       'border-top: 0px;}')
                dummyBox.setLayout(stateLayout)
                rowLayout.addWidget(dummyBox)
                viewLbl.clicked.connect(partial(self.toggleView, dummyBox))
                layout.addLayout(rowLayout)
        else:
            stateLayout = layout

        if len(state.getNamespace().getParams()) > 0:
            self.displayStateParams(state, stateLayout, stateUIs, childTitleLblStyleSheet)

        if len(state.getChildren()) > 0:
            childStatesTitleLbl = QLabel('Child States:')
            childStatesTitleLbl.setStyleSheet(childTitleLblStyleSheet)
            stateLayout.addWidget(childStatesTitleLbl)
            stateUIs.append(childStatesTitleLbl)
            for child in state.getChildren():
                if root:
                    self.addStates(stateLayout, child, None)
                else:
                    self.addStates(stateLayout, child, stateUIs)
            dummyBox = QGroupBox()
            dummyBox.setStyleSheet('QGroupBox {background-color: white; border: 0px;}')
            dummyBox.setMinimumWidth(100)
            dummyBox.setFixedHeight(0)
            stateLayout.addWidget(dummyBox)

    def setStateEnabled(self, stateUIs, state):
        for UI in stateUIs:
            if UI is not None:
                if type(UI) == list:
                    self.setStateEnabled(UI, state)
                else:
                    UI.setEnabled(state)
                    if isinstance(UI, QCheckBox):
                        UI.setChecked(state)

    def toggleView(self, widget):
        label = self.sender()
        if widget.isHidden():
            widget.setHidden(False)
            label.setText(u'\u25BE')
        else:
            widget.setHidden(True)
            label.setText(u'\u25B8')
            label.setMinimumWidth(20)

    def displayStateParams(self, state, stateLayout, stateUIs, childTitleLblStyleSheet):
        rowLayout = QHBoxLayout()
        rowLayout.setAlignment(Qt.AlignLeft)
        paramTitleLbl = QLabel('Parameters')
        paramTitleLbl.setStyleSheet(childTitleLblStyleSheet)
        rowLayout.addWidget(paramTitleLbl)
        viewLbl = ClickableLabel(u'\u25BE')
        viewLbl.setMinimumWidth(20)
        rowLayout.addWidget(viewLbl)
        stateLayout.addLayout(rowLayout)
        stateUIs.append(paramTitleLbl)

        rowLayout = QHBoxLayout()
        rowLayout.addSpacing(10)
        paramLayout = QVBoxLayout()
        paramLayout.setDirection(QBoxLayout.TopToBottom)
        paramLayout.setAlignment(Qt.AlignTop)
        dummyBox = QGroupBox()
        dummyBox.setStyleSheet('QGroupBox {background-color: white; border: 0px; border-radius:3px;}')
        dummyBox.setLayout(paramLayout)
        rowLayout.addWidget(dummyBox)
        viewLbl.clicked.connect(partial(self.toggleView, dummyBox))
        stateLayout.addLayout(rowLayout)

        titleLblStyleSheet = 'QLabel {font: italic;}'
        rowLayout = QHBoxLayout()
        titleLbl = QLabel('')
        titleLbl.setStyleSheet(titleLblStyleSheet)
        titleLbl.setFixedWidth(20)
        rowLayout.addWidget(titleLbl)
        titleLbl = QLabel('Name')
        titleLbl.setStyleSheet(titleLblStyleSheet)
        titleLbl.setFixedWidth(150)
        rowLayout.addWidget(titleLbl)
        stateUIs.append(titleLbl)
        rowLayout.addSpacing(5)
        titleLbl = QLabel('Type')
        titleLbl.setStyleSheet(titleLblStyleSheet)
        titleLbl.setFixedWidth(60)
        rowLayout.addWidget(titleLbl)
        stateUIs.append(titleLbl)
        rowLayout.addSpacing(5)
        titleLbl = QLabel('Value')
        titleLbl.setStyleSheet(titleLblStyleSheet)
        titleLbl.setFixedWidth(100)
        rowLayout.addWidget(titleLbl)
        stateUIs.append(titleLbl)
        rowLayout.addSpacing(5)
        titleLbl = QLabel('Description')
        titleLbl.setStyleSheet(titleLblStyleSheet)
        titleLbl.setMinimumWidth(300)
        rowLayout.addWidget(titleLbl)
        stateUIs.append(titleLbl)

        paramLayout.addLayout(rowLayout)

        for param in state.getNamespace().getParams():
            paramUIs = self.addParam(paramLayout, param)
            stateUIs.append(paramUIs)

    def addParam(self, layout, param):
        paramUIs = []
        rowLayout = QHBoxLayout()
        showLbl = ClickableLabel(u'\U0001F441')
        showLbl.setToolTip("View")
        showLbl.clicked.connect(partial(self.showParam, param))
        showLbl.setFixedWidth(20)
        rowLayout.addWidget(showLbl)
        nameLbl = ElidedLabel(param.name)
        nameLbl.setToolTip(param.name)
        nameLbl.setFixedWidth(150)
        rowLayout.addWidget(nameLbl)
        paramUIs.append(nameLbl)
        rowLayout.addSpacing(5)
        typeLbl = ElidedLabel(param.type)
        typeLbl.setFixedWidth(60)
        rowLayout.addWidget(typeLbl)
        paramUIs.append(typeLbl)
        rowLayout.addSpacing(5)
        valueLbl = ElidedLabel(param.value)
        valueLbl.setToolTip(param.value)
        valueLbl.setFixedWidth(100)
        rowLayout.addWidget(valueLbl)
        paramUIs.append(valueLbl)
        rowLayout.addSpacing(5)
        descLbl = ElidedLabel(param.desc)
        descLbl.setAlignment(Qt.AlignTop)
        descLbl.setFixedHeight(17)
        descLbl.setToolTip(param.desc)
        descLbl.setMinimumWidth(300)
        rowLayout.addWidget(descLbl)
        paramUIs.append(descLbl)
        layout.addLayout(rowLayout)
        return paramUIs

    def showParam(self, param):
        paramPropDialog = ParamPropDialog(param=param, modify=False)
        paramPropDialog.exec_()

    def removeStates(self, parentState):
        remList = []
        for child in parentState.getChildren():
            if not self.checkBoxList[child.id - 1].isChecked():
                remList.append(child)
            else:
                self.removeStates(child)
        for child in remList:
            for transition in child.getDestTransitions():
                transition.origin.removeOriginTransition(transition)
            parentState.removeChild(child)

    def doneClicked(self):
        self.removeStates(self.file[0])
        self.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = ImportedParamsDialog('Parameters')
    dialog.exec_()