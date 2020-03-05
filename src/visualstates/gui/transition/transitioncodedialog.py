'''
   Copyright (C) 1997-2017 JDERobot Developers Team

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

   Authors : Okan Asik (asik.okan@gmail.com)

  '''
import sys

from PyQt5.Qsci import QsciScintilla, QsciLexerPython, QsciLexerCPP
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase, QFontMetrics, QColor
from PyQt5.QtWidgets import QDialog, QTextEdit, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QApplication, \
    QRadioButton, QGroupBox, QMessageBox

from visualstates.gui.transition.transitiontype import TransitionType


class TransitionCodeDialog(QDialog):
    codeChanged = pyqtSignal('int', 'QString', 'QString')

    def __init__(self, name, transition):
        super(QDialog, self).__init__()
        self.transition = transition
        self.setWindowTitle(name)
        self.resize(800, 600)

        self.codeEdit = QsciScintilla()
        self.codeEdit.setText(self.transition.getCode())
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.codeEdit.setFont(fixedWidthFont)
        fontmetrics = QFontMetrics(fixedWidthFont)
        self.codeEdit.setMarginWidth(0, fontmetrics.width("000"))
        self.codeEdit.setMarginLineNumbers(0, True)
        self.codeEdit.setMarginsBackgroundColor(QColor("#cccccc"))

        self.codeEdit.setBraceMatching(QsciScintilla.SloppyBraceMatch)
        self.codeEdit.setCaretLineVisible(True)
        self.codeEdit.setCaretLineBackgroundColor(QColor("#ffe4e4"))
        lexer = QsciLexerPython()
        lexer.setDefaultFont(fixedWidthFont)
        self.codeEdit.setLexer(lexer)
        self.codeEdit.SendScintilla(QsciScintilla.SCI_SETHSCROLLBAR, 0)
        self.codeEdit.setUtf8(True)

        self.codeEdit.setTabWidth(4)
        self.codeEdit.setIndentationsUseTabs(True)
        self.codeEdit.setIndentationGuides(True)
        self.codeEdit.setTabIndents(True)
        self.codeEdit.setAutoIndent(True)
        self.codeEdit.textChanged.connect(self.codeChangedListener)

        self.language = 'python'
        self.pythonButton = QRadioButton('Python')
        self.pythonButton.setChecked(True)
        self.pythonButton.clicked.connect(self.pythonClicked)
        self.cppButton = QRadioButton('C++')
        self.cppButton.clicked.connect(self.cppClicked)

        codeLanguageContainer = QWidget()
        hLayout0 = QHBoxLayout()
        hLayout0.addWidget(self.pythonButton)
        hLayout0.addWidget(self.cppButton)
        codeLanguageContainer.setLayout(hLayout0)

        self.temporalButton = QRadioButton('Temporal', self)
        self.temporalButton.toggled.connect(self.temporalToggled)
        self.conditionalButton = QRadioButton('Conditional', self)
        self.conditionalButton.toggled.connect(self.conditionalToggled)

        radioButtonContainer = QGroupBox()
        radioButtonContainer.setTitle('Transition Type')
        vLayout = QVBoxLayout()
        vLayout.addWidget(self.temporalButton)
        vLayout.addWidget(self.conditionalButton)
        radioButtonContainer.setLayout(vLayout)

        self.transitionTypeCode = QTextEdit()
        self.transitionTypeCode.setFont(fixedWidthFont)
        self.transitionTypeCode.textChanged.connect(self.transitionTypeCodeChanged)

        self.transitionGroupBox = QGroupBox()
        self.transitionGroupBox.setTitle('Temporal (number in ms)')
        h3Layout = QHBoxLayout()
        h3Layout.addWidget(self.transitionTypeCode)
        self.transitionGroupBox.setLayout(h3Layout)

        typeContainer = QWidget()
        h2Layout = QHBoxLayout()
        h2Layout.addWidget(radioButtonContainer)
        h2Layout.addWidget(self.transitionGroupBox)
        typeContainer.setLayout(h2Layout)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(typeContainer)
        verticalLayout.addWidget(codeLanguageContainer)
        verticalLayout.addWidget(self.codeEdit)
        self.setLayout(verticalLayout)

        if self.transition.getType() == TransitionType.CONDITIONAL:
            self.conditionalButton.setChecked(True)
            self.conditionalToggled()
        elif self.transition.getType() == TransitionType.TEMPORAL:
            self.temporalButton.setChecked(True)
            self.temporalToggled()

    def codeChangedListener(self):
        transitionType = TransitionType.CONDITIONAL
        if self.temporalButton.isChecked():
            transitionType = TransitionType.TEMPORAL

        self.codeChanged.emit(transitionType, self.transitionTypeCode.toPlainText(), self.codeEdit.text())

    def transitionTypeCodeChanged(self):
        # validate the input
        typeCode = self.transitionTypeCode.toPlainText()
        transitionType = TransitionType.CONDITIONAL
        if self.temporalButton.isChecked():
            transitionType = TransitionType.TEMPORAL
            if typeCode and not typeCode.isdigit():
                QMessageBox.warning(self, 'Input Error', 'Please input an integer in the Temporal box')
                return
        if not typeCode:
            typeCode = '0'
        self.codeChanged.emit(transitionType, typeCode, self.codeEdit.text())

    def temporalToggled(self):
        if (self.temporalButton.isChecked()):
            self.transitionGroupBox.setTitle('Temporal (number in ms)')
            self.transitionTypeCode.setPlainText(str(self.transition.getTemporalTime()))
            self.codeChanged.emit(TransitionType.TEMPORAL, self.transitionTypeCode.toPlainText(), self.codeEdit.text())

    def conditionalToggled(self):
        if (self.conditionalButton.isChecked()):
            self.transitionGroupBox.setTitle('Condition (evaluates to true or false)')
            self.transitionTypeCode.setPlainText(self.transition.getCondition())
            self.codeChanged.emit(TransitionType.CONDITIONAL, self.transitionTypeCode.toPlainText(), self.codeEdit.text())

    def pythonClicked(self):
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        lexer = QsciLexerPython()
        lexer.setDefaultFont(fixedWidthFont)
        self.codeEdit.setLexer(lexer)
        self.language = 'python'

    def cppClicked(self):
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        lexer = QsciLexerCPP()
        lexer.setDefaultFont(fixedWidthFont)
        self.codeEdit.setLexer(lexer)
        self.language = 'cpp'

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = TransitionCodeDialog('Rename', 'Hello World')
    dialog.exec_()
