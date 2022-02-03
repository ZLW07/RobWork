from PySide2 import QtWidgets, QtGui
import sys
from sdurws import *
from sdurw import *
from pprint import pprint

#find widget
def findWidget():
    all_widgets = QtWidgets.QApplication.allWidgets()
    window = None
    for widget in all_widgets:
        if str(widget.objectName()) == sys.argv[0]:
            window = widget
            break  
    return window
#done finding widget

def message_box():
    QtWidgets.QMessageBox.information(None, 'Information', 'You are about to close RobWorkStudio',QtWidgets.QMessageBox.Ok)
    QtWidgets.QApplication.instance().quit()

def addQ(path,time,q,device,state):
    device.setQ(q,state)
    tstate = TimedState(time,state)
    path.push_back(tstate)

def move_robot():
    rwstudio = getRobWorkStudioFromQt()

    state = rwstudio.getState()
    device = rwstudio.getWorkCell().findDevice("UR5_Left")
    path = ownedPtr(PathTimedState())
    path.clear()

    addQ(path, 0, Q(6,0,0.1,0,0,0,0), device, state)
    addQ(path, 1, Q(6,0,0.1,0,0,0,0), device, state)
    addQ(path, 1, Q(6,0,0.1,0,0,0,0), device, state)
    addQ(path, 2, Q(6,0,0.2,0,0,0,0), device, state)
    addQ(path, 3, Q(6,0,0.3,0,0,0,0), device, state)
    addQ(path, 4, Q(6,0,0.4,0,0,0,0), device, state)
    addQ(path, 5, Q(6,0,0.5,0,0,0,0), device, state)
    addQ(path, 6, Q(6,0,0.6,0,0,0,0), device, state)

    rwstudio.setTimedStatePath(path);


#argv[0]= widgetName
#argv[1]= pluginName
if __name__ == '__main__':
    window = findWidget()                       # find the plugin widget
    l = window.layout()                         # get the layout the default is a QGridLayout
    l.setColumnStretch(0,1)

    #make btn to move robot
    btn_move = QtWidgets.QPushButton('moveRobot')    # make btn
    btn_move.clicked.connect(move_robot)             # connect btn with function
    l.addWidget(btn_move,0,0)                        # add btn to layout

    #make btn to quit rws
    btn_quit = QtWidgets.QPushButton('Quit')    # make btn
    btn_quit.clicked.connect(message_box)       # connect btn with function
    l.addWidget(btn_quit,1,0)                   # add btn to layout

    l.setRowStretch(2,1)                        #Push Stuff to top

    

    

#Adde the following lines to RobworkStudio.ini to load this plugin
#pluginName\DockArea=1
#pluginName\Filename=PythonPlugin
#pluginName\Path=/path/to/RobWork/RobWorkStudio/example/python
#pluginName\Visible=false