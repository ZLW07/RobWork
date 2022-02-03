from sdurws import *
import sys
from PySide2 import QtCore
import copy

class cpp_link(QtCore.QObject):
    def __init__(self):
        self.plugin={"reserved" : None}
    def register_plugin(self,aPlugin):
        if self.plugin[self.get_widget_name()] == None:
            print("Plugin registered as: ", self.get_widget_name())
            self.plugin[self.get_widget_name()]=aPlugin
        else:
            raise Exception("Only One Plugin can be registered")       
    def stateChanged(self):
        for key in self.plugin:
            stateChangedListener = getattr(self.plugin[key],"stateChangedListener",None)
            if callable(stateChangedListener):
                stateChangedListener(getRobWorkStudioFromQt().getState())
    def openWorkCell(self):
        for key in self.plugin:
            open = getattr(self.plugin[key],"open",None)
            if callable(open):
                open(getRobWorkStudioFromQt().getWorkCell())
    def closeWorkCell(self):
        for key in self.plugin:
            close = getattr(self.plugin[key],"close",None)
            if callable(close):
                close()
    def new_widget(self,name):
        self.latest_widget = name
        self.plugin[name]=None
    def get_widget_name(self):
        return copy.deepcopy(self.latest_widget)

rws_cpp_link = cpp_link()

class rwsplugin(QtCore.QObject):
    def __init__(self,link):
        self.rws_cpp_link = link
        self.rwstudio = getRobWorkStudioFromQt()
        self.widget_name = rws_cpp_link.get_widget_name()
    def getWidget(self):
        all_widgets = QtWidgets.QApplication.allWidgets()
        window = None
        for widget in all_widgets:
            if str(widget.objectName()) == self.widget_name:
                window = widget
                break  
        return window
    def getRobWorkStudio(self):
        return self.rwstudio
    def getWorkCell(self):
        return getRobWorkStudio().getWorkCell() 
    def log(self):
        return self.rwstudio.log()
