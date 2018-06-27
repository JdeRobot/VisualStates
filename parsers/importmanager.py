'''
   Copyright (C) 1997-2018 JDERobot Developers Team

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

   Authors : Pushkal Katara (katarapushkal@gmail.com)

  '''

from PyQt5.QtWidgets import QFileDialog
from configs.config import RosConfig

class ImportManager():
    """
    Functionality:
    Import PreBuild State into Current State
    Verify Configurations
    Verify Libraries
    Verify Functions
    Verify Variables

    :param rootState: Root Imported State
    :param config: Configurations of Imported State
    :param libraries: Libraries of Imported State
    :param functions: Functions of Imported State
    :param variables: Variables of Imported State

    Returns list of States which needs to be Imported
    """
    def __init__(self):
        self.JDEROBOTCOMM = 0
        self.ROS = 1

    def updateAuxiliaryData(self, file, klass):
        """Wrapper upon all update functions"""
        importedState = self.updateActiveState(file[0], klass.automataScene.getStateIndex(), klass.activeState)
        config = self.updateConfigs(file[1], klass.config)
        libraries = self.updateLibraries(file[2], klass.libraries)
        functions = self.updateFunctions(file[3], klass.functions)
        variables = self.updateVariables(file[4], klass.variables)
        return importedState, config, libraries, functions, variables

    def updateFunctions(self, newFunctions, functions):
        """Updates existing functions with imported functions"""
        if functions == newFunctions:
            return functions
        else:
            return functions+ "\n" + newFunctions

    def updateVariables(self, newVariables, variables):
        """Updates existing variables with imported variables"""
        if variables == newVariables:
            return variables
        else:
            return variables+newVariables

    def updateLibraries(self, newLibraries, libraries):
        """Updates existing libraries with imported libraries"""
        for lib in newLibraries:
            if lib not in libraries:
                libraries.append(lib)
        return libraries

    def updateConfigs(self, newConfig, config):
        """Updates Existing Configurations with imported Configurations"""
        if newConfig:
            if config and newConfig.type == config.type:
                if newConfig.type == self.ROS:
                    config.updateROSConfig(newConfig)
                elif newConfig.type == self.JDEROBOTCOMM:
                    config.updateJDERobotCommConfig(newConfig)
            else:
                if newConfig.type == self.ROS:
                    config = RosConfig()
                    config.updateROSConfig(newConfig)
                elif newConfig.type == self.JDEROBOTCOMM:
                    config = JdeRobotConfig()
                    config.updateJDERobotCommConfig(newConfig)
        return config

    def updateActiveState(self, importState, stateID, activeState):
        """Updates Parent State with States to be imported"""
        importState = self.updateIDs(importState, stateID)
        for state in importState.getChildren():
            activeState.addChild(state)
            state.setParent(activeState)
        return importState

    def updateIDs(self, importState, stateID):
        """ Wrapper upon UpdateStateIDs """
        self.updateStateIDs(importState, stateID)
        return importState

    def updateStateIDs(self, importState, stateID):
        """ Assign New IDs to Imported State Data Recursively """
        for child in importState.getChildren():
            child.setID(stateID)
            stateID += 1
            self.updateStateIDs(child, stateID)
