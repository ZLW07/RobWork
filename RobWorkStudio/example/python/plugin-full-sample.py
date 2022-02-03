#!/usr/bin/python3
from PySide2 import QtWidgets, QtGui, QtCore
from sdurws import *
from sdurw import *
from sdurw_simulation import *
from sdurw_proximitystrategies import *
from sdurw_pathplanners import *
from sdurw_opengl import *

import numpy as np
import cv2 #This might cause segfault unless you have use the "opencv-contrib-python-headless" pkg from pip

import copy

class Plugin(rwsplugin):
    def __init__(self,link):
        super().__init__(link)
        self.setupUI()
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.callBack_timer)

        self.btn_getImg.clicked.connect(self.callBack_btn_getImg)             
        self.btn_getScan.clicked.connect(self.callBack_btn_getScan)             
        self.btn_calculatePath.clicked.connect(self.callBack_btn_calculatePath)             
        self.btn_runPath.clicked.connect(self.callBack_btn_runPath)            

        self.framegrapper = None
        self.cameras = ("Camera_Right", "Camera_Left")
        self.cameras25D = ("Scanner25D",)  

        # initialize
        wc = WorkCellLoaderFactory.load("/home/kalor/Documents/Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml")
        self.getRobWorkStudio().setWorkCell(wc)

        # variables

        self.wc = None
        self.state = None
        self.textureRender = ownedPtr(RenderImage())
        self.bgRender = ownedPtr(RenderImage())
        self.framegrapper = None
        self.framegrapper25D = None
        self.device = None
        self.path = PathQ()
        self.step = 0

        #
    def setupUI(self):
        self.widget = super().getWidget()
        self.layout = self.widget.layout()  
        self.layout.setColumnStretch(0,1)

        self.btn_getImg = QtWidgets.QPushButton('Get Image')
        self.btn_getScan = QtWidgets.QPushButton('Get Scan') 
        self.btn_calculatePath = QtWidgets.QPushButton('Calculate Path')
        self.btn_runPath = QtWidgets.QPushButton('Run Path')  
        self.spinbox = QtWidgets.QSpinBox()
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.label = QtWidgets.QLabel("Label")

        self.layout.addWidget(self.btn_getImg,0,0)        
        self.layout.addWidget(self.btn_getScan,1,0)  
        self.layout.addWidget(self.btn_calculatePath,2,0)  
        self.layout.addWidget(self.btn_runPath,3,0)
        self.layout.addWidget(self.spinbox,4,0)
        self.layout.addWidget(self.slider,5,0)
        self.layout.addWidget(self.label,6,0)
        self.layout.setRowStretch(7,1) 

        #
    def open(self,workcell):
        super().log().info().write("Open \n")

        self.wc = workcell
        self.state = self.wc.getDefaultState()

        super().log().info().write(workcell.getFilename() + "\n")

        if (not self.wc == None) and (not self.wc.isNull()):
            
            # Create a GLFrameGrabber if there is a camera frame with a Camera property set
            cameraFrame = self.wc.findFrame(self.cameras[0])
            if not cameraFrame == None:
                if cameraFrame.getPropertyMap().has("Camera"):
                    cameraParam = cameraFrame.getPropertyMap().getString("Camera")
                    fovy, width, height = cameraParam.split(' ')
                    self.framegrapper = GLFrameGrabber(int(width),int(height),float(fovy))
                    gldrawer = super().getRobWorkStudio().getView().getSceneViewer()
                    self.framegrapper.init(gldrawer)
            
            cameraFrame25D = self.wc.findFrame(self.cameras25D[0])
            if not cameraFrame25D == None:
                if cameraFrame25D.getPropertyMap().has("Scanner25D"):
                    cameraParam = cameraFrame25D.getPropertyMap().getString("Scanner25D")
                    fovy, width, height = cameraParam.split(' ')
                    self.framegrapper25D = GLFrameGrabber25D(int(width),int(height),float(fovy))
                    gldrawer = super().getRobWorkStudio().getView().getSceneViewer()
                    self.framegrapper25D.init(gldrawer)

            self.device = self.wc.findDevice("UR-6-85-5-A")
            self.step=-1

        #
    def close(self):
        super().log().info().write("CLOSE \n")

        #Stop the timer
        self.timer.stop()
        self.framegrapper = None
        self.wc = None

        #
    def toOpenCVImage(self, rw_img):
        img = np.zeros((rw_img.getWidth(),rw_img.getHeight(),rw_img.getNrOfChannels()),dtype=np.uint8)

        for x in range(0,rw_img.getWidth()):
            for y in range(0,rw_img.getHeight()):
                for c in range(0, rw_img.getNrOfChannels()):
                    img[x,y,c]=rw_img.getPixelValuei(x,y,c)

        return img

        #
    def callBack_btn_getImg(self):
        self.getImage()

        #
    def callBack_btn_getScan(self):
        self.get25DImage()

        #
    def callBack_btn_calculatePath(self):
        self.timer.stop()
        Math.seed()
        extend = 0.05
        maxTime = 60
        From = Q(6, 1.571, -1.572, -1.572, -1.572, 1.571, 0)
        To = Q(6, 1.847, -2.465, -1.602, -0.647, 1.571, 0) # From pose estimation
        self.createPathRRTConnect(From,To,extend,maxTime)
        #
    def callBack_btn_runPath(self):
        super().log().info().write("Button 1\n")
        # Toggle the timer on and off 
        if not self.timer.isActive():
            self.timer.start(100) # run 10 Hz
            self.step = 0
        else:
            self.step = 0
        
        #
    def getImage(self):
        if not self.framegrapper == None:
            for i in range(0,len(self.cameras)):
                cameraFrame = self.wc.findFrame(self.cameras[i])
                self.framegrapper.grab(cameraFrame,self.state)

                img = self.framegrapper.getImage()

                cvImgTmp = self.toOpenCVImage(img)
                cvImg = None 
                cvImg=cv2.transpose(cvImgTmp)

                height, width, channel = cvImg.shape
                bytesPerLine = 3 * width
                qimg = QtGui.QImage(cvImg.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
                pix = QtGui.QPixmap.fromImage(qimg)
                self.label.setPixmap(pix)


        #
    def get25DImage(self):
        print("print get 25DImg", self.framegrapper25D)
        if not self.framegrapper25D == None:
            for cam25d in self.cameras25D:
                print("cam cam25d")

                cameraFrame25D = self.wc.findFrame(cam25d)
                self.framegrapper25D.grab(cameraFrame25D, self.state)

                img = self.framegrapper25D.getImage()
                
                PointCloud.savePCD(img,cam25d+".pcd")
        #        
    def callBack_timer(self):
        if 0 <= self.step and self.step < self.path.size():
            self.device.setQ(self.path.elem(self.step),self.state)
            super().getRobWorkStudio().setState(self.state)
            self.step = self.step + 1
        
        #
    def stateChangedListener(self,state):
        self.state = state

        #
    def checkCollisions(self, device, testState, detector, q):
        data = FramePairVector()

        device.setQ(q,testState)
        colFrom = detector.inCollision(testState,data)

        if colFrom:
            print("Configuration in collision: "+ str(q) + "\n")
            print("Colliding Frames: \n")
            
            for fPair in data:
                print( fPair.first.getName() + " " + fPair.second.getName())

        return not colFrom
        
        #
    def createPathRRTConnect(self, From, To , extend, maxTime):
        self.device.setQ(From, self.state)
        CDevice = self.device.asDeviceCPtr()
        super().getRobWorkStudio().setState(self.state)
        detector    = sdurw.ownedPtr(CollisionDetector(self.wc, ProximityStrategyFactory.makeDefaultCollisionStrategy()))
        constraint  = PlannerConstraint.make(detector,CDevice,self.state)
        sampler     = QSampler.makeConstrained(QSampler.makeUniform(CDevice),constraint.getQConstraintPtr().asCPtr())
        metric      = MetricFactory().makeEuclideanQ()
        planner     = RRTPlanner.makeQToQPlanner(constraint,sampler,metric,extend,RRTPlanner.RRTConnect)

        self.path.clear()
        if not self.checkCollisions(self.device,self.state,detector,From):
            print(From, "is in colission")
        if not self.checkCollisions(self.device,self.state,detector,To):
            print(To, "is in colission")

        t = Timer()
        t.resetAndResume()
        planner.query(From,To,self.path,maxTime)
        t.pause()

        if t.getTime() >= maxTime:
            print("Notice: max time of ", maxTime ," seconds reached.")
        
        duration = 10

        if self.path.size() == 2:
            linInt = LinearInterpolatorQ(From, To, duration)
            tempQ = PathQ()
            for i in range(0,duration+1):
                tempQ.push_back(linInt.x(i))
            self.path=tempQ

if __name__ == '__main__':
    pl = Plugin(rws_cpp_link)
    rws_cpp_link.register_plugin(pl)
    