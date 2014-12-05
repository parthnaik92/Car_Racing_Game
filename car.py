# -*- coding: utf_8 -*-
from random import randint, random
from direct.showbase import DirectObject 
import math
from pandac.PandaModules import *
from direct.gui.OnscreenImage import OnscreenImage
#from direct.gui.OnscreenText import OnscreenText
loadPrcFileData("editor-startup", "sync-video #t")
loadPrcFileData("editor-startup", "show-frame-rate-meter #t")
from direct.directbase import DirectStart

class World():
    def __init__(self):
        #self.infoText = OnscreenText(text = '0', pos = (-0.9, 0.9), scale = 0.07, mayChange=1)

        #lights
        self.ambientLight = render.attachNewNode( AmbientLight( "ambientLight" ) )
        self.ambientLight.node().setColor( Vec4( .8, .8, .8, 1 ) )
        self.PointLight = camera.attachNewNode( PointLight( "PointLight" ) )
        self.PointLight.node().setColor( Vec4( 0.8, 0.8, 0.8, 1 ) )
        self.PointLight.node().setAttenuation( Vec3( .1, 0.04, 0.0 ) ) 
        render.setLight( self.ambientLight )
        render.setLight( self.PointLight )

        #set physic world
        self.world = OdeWorld()
        self.world.setGravity(0, 0, -9.81)
        #Adjustment of interaction surfaces
        self.world.initSurfaceTable(4)
        # (surfaceId1, surfaceId2, mu, bounce, bounce_vel, soft_erp, soft_cfm, slip, dampen)
        self.world.setSurfaceEntry(0, 0, 0.8, 0.0, 10    , 0.9, 0.00001, 100, 0.002)
        self.world.setSurfaceEntry(0, 1, 0.8, 0.1, 10, 0.8, 0.00005, 0, 1)        
        self.world.setSurfaceEntry(0, 2, 0.9, 0.1, 10, 0.8, 0.00005, 0, 1)        
        self.world.setSurfaceEntry(3, 1, 0.4, 0.2, 10, 0.7, 0.00005, 0, 1)        
        self.world.setSurfaceEntry(3, 2, 0.4, 0.2, 10, 0.7, 0.00005, 0, 1)        

        #Create ODE space; add contact group for store contact joints
        self.space = OdeSimpleSpace()
        self.space.setAutoCollideWorld(self.world)
        self.contactgroup = OdeJointGroup()
        self.space.setAutoCollideJointGroup(self.contactgroup)
        self.world.setQuickStepNumIterations(10)

        #Create basic box for copy later
        box = loader.loadModel("box")
        tex = loader.loadTexture('box.png')
        ts = TextureStage('ts')
        ts.setMode(TextureStage.MModulate)
        box.setTexture(ts,tex)
        #list of boxes
        self.boxes = []
        #generate some random boxes
        for i in range(randint(10, 20)):
            #randomize size
            sx=(random()+0.5) * 3
            sy=(random()+0.5) * 3
            sz=(random()+0.5) * 3
            #copy of the basic box
            boxNP = box.copyTo(render)
            #random position,
            boxNP.setPos(randint(-40, 40), randint(-40, 40), 20 + randint(0,20))
            #color
            boxNP.setColor(random()*2, random()*2, random()*2, 1)
            #and orientation
            boxNP.setHpr(randint(-45, 45), randint(-45, 45), randint(-45, 45))
            #set size
            boxNP.setScale(sx,sy,sz)
            #Set physic body for our box
            boxBody = OdeBody(self.world)
            M = OdeMass()
            M.setBox(1.5, 1, 1, 1)
            boxBody.setMass(M)
            #sync box and body position and orientation
            boxBody.setPosition(boxNP.getPos(render))
            boxBody.setQuaternion(boxNP.getQuat(render))
            #set ODE geometry for body. Panda's visual geometry-mesh and 
            #ODE geometry is different things
            boxGeom = OdeBoxGeom(self.space, sx,sy,sz)
            boxGeom.setBody(boxBody)
            # Add box and body to list for use later
            self.boxes.append((boxNP, boxBody))
##        self.cm = CardMaker("ground")
##        self.cm.setUvRange(Point2(0, 0), Point2(100, 100))
##        self.cm.setFrame(-2000, 2000, -2000, 2000)
##        self.ground = render.attachNewNode(self.cm.generate())
##        self.ground.setPos(0, 0, 0); self.ground.lookAt(0, 0, -2)
##        ts = TextureStage('ts')
##        self.ground.setTexture(ts,tex)
        self.groundGeom = OdePlaneGeom(self.space, Vec4(0, 0, 1, 0))
        #the border for the road
        b1 = loader.loadModel('border.egg')
        b1.setScale(8)
        b1.flattenStrong()  #apply transform before read vertex info for create ODE geometry
        b1Trimesh = OdeTriMeshData(b1)
        self.b1Geom = OdeTriMeshGeom(self.space, b1Trimesh)
        self.space.setSurfaceType(self.b1Geom, 3)
        b1.reparentTo(render)  
        #Our border must collide with the box and car, but never with the road
        self.b1Geom.setCollideBits(BitMask32(0x00000002))
        self.b1Geom.setCategoryBits(BitMask32.allOff())

        #Load the road similar the border
        road = loader.loadModel('track.egg')
        road.setScale(8)
        road.flattenLight() 
        roadTrimesh = OdeTriMeshData(road, False)
        self.roadGeom = OdeTriMeshGeom(self.space, roadTrimesh)
        road.reparentTo(render)
        self.roadGeom.setCollideBits(BitMask32(0x00000002))
        self.roadGeom.setCategoryBits(BitMask32.allOff())      
        print render.ls() 
        #car
        self.car=Car(self.world,self.space,Vec3(0,0,5))
       
        base.disableMouse()
        #add simulation to the task manager
        taskMgr.doMethodLater(0.5, self.simulationTask, "Physics Simulation")
    
    #simulation procedure
    def simulationTask(self,task):
        iterations = 5
        #We limit the maximum time not to receive explosion of physic system if application stuck
        dt=globalClock.getDt()
        if dt>0.02: dt=0.02
        dt=dt / iterations * 3
        
        #Some iterations for the more stable simulation
        for i in xrange(iterations):
          self.world.quickStep(dt)
          cc=self.space.autoCollide()        
        #Sync the box with the bodies
        for np, body in self.boxes:
            np.setPos(render, body.getPosition())
            np.setQuat(render,Quat(body.getQuaternion()))
        self.contactgroup.empty() #clear contacts before next step
        self.car.Sync()#sync the car
        return task.cont

#Car class
class Car():
    def __init__(self,world,space,pos):
        #variables
        self.world=world
        self.world.setContactSurfaceLayer(0.01)
        self.space=space
        self.turn=False
        self.turnspeed=0.0
        self.turnangle=0.0
        self.carOrientation=1
        self.acceleration=False
        self.maxSpeed=0
        self.accForce=0
        #Body of the our car - similar the boxes
        self.box = loader.loadModel("car")
        self.box.setPos(pos)
        self.box.setColor(1,0.5,0.5)
        self.box.reparentTo(render)     
        self.body=OdeBody(self.world)
        M = OdeMass()
        M.setBox(4, 1.8, 4, 0.5)
        self.body.setMass(M)
        self.body.setPosition(self.box.getPos(render))
        self.bodyGeom = OdeBoxGeom(self.space, 1,3,1)
        self.bodyGeom.setBody(self.body)
        
        self.joints=[] #suspensions
        self.wheelsbody=[] #wheels body
        self.wheelsgeom=[] #wheels geometry
        self.wheels=[]     #wheels visualisation
        for i in range(4):
            #set physic of the wheel
            self.wheelsbody.append(OdeBody(self.world))
            M = OdeMass()
            M.setCylinder(2,2,1, 0.4)
            self.wheelsbody[i].setMass(M)
            self.wheelsbody[i].setQuaternion(Quat(0.7,0,0.7,0))
            self.wheelsbody[i].setFiniteRotationMode(1)
            #self.wheelsgeom.append(OdeCylinderGeom(self.space, 1,0.4))
            self.wheelsgeom.append(OdeSphereGeom(self.space, 1))
            self.wheelsgeom[i].setBody(self.wheelsbody[i])  
            self.wheelsgeom[i].setCategoryBits(BitMask32(0x00000002))
            #add hinge2 joint, wich simulate suspension
            self.joints.append(OdeHinge2Joint(self.world))
            self.joints[i].attachBodies(self.body,self.wheelsbody[i])
            #min/max angle for the wheel. Set min=max for stable turn
            self.joints[i].setParamHiStop(0, 0.0)
            self.joints[i].setParamLoStop(0, 0.0)
            #Error reduction parameter of suspension
            self.joints[i].setParamSuspensionERP(0, 0.9)
            #Blending of forces - in this case influences rigidity of a suspension
            self.joints[i].setParamSuspensionCFM(0, 0.001)            
            #self.joints[i].setParamFudgeFactor(0,0.1)
            #axis of joint: set one - vertical, and one - horisontal
            self.joints[i].setAxis1(0,0,1)
            self.joints[i].setAxis2(1,0,0)            
            #visual mesh of wheel
            self.wheels.append(loader.loadModelCopy("wheel"))
            self.wheels[i].setColor(1,0.5,0.5)
            self.wheels[i].setScale(1,1,2)
            self.wheels[i].reparentTo(render)
        
        wheelDistance = 3.1 #1.8
        bodyDistance = 5.2 # 1.1
        #bodyHeight = 2.5
        bodyHeight= -1.5#car add
        #set wheels to start position
        self.wheelsbody[0].setPosition(pos.getX()-wheelDistance,pos.getY()+bodyDistance,pos.getZ()+bodyHeight)
        self.wheelsbody[1].setPosition(pos.getX()-wheelDistance,pos.getY()-bodyDistance,pos.getZ()+bodyHeight)
        self.wheelsbody[2].setPosition(pos.getX()+wheelDistance,pos.getY()+bodyDistance,pos.getZ()+bodyHeight)
        self.wheelsbody[3].setPosition(pos.getX()+wheelDistance,pos.getY()-bodyDistance,pos.getZ()+bodyHeight)
        #set joints to start position
        self.joints[0].setAnchor(Vec3(pos.getX()-(wheelDistance-0.2),pos.getY()+bodyDistance,pos.getZ()+bodyHeight))    
        self.joints[1].setAnchor(Vec3(pos.getX()-(wheelDistance-0.2),pos.getY()-bodyDistance,pos.getZ()+bodyHeight))
        self.joints[2].setAnchor(Vec3(pos.getX()+(wheelDistance-0.2),pos.getY()+bodyDistance,pos.getZ()+bodyHeight))
        self.joints[3].setAnchor(Vec3(pos.getX()+(wheelDistance-0.2),pos.getY()-bodyDistance,pos.getZ()+bodyHeight))
        #Set surface types for the wheels
        self.space.setSurfaceType(self.wheelsgeom[0],1)
        self.space.setSurfaceType(self.wheelsgeom[2],1)
        self.space.setSurfaceType(self.wheelsgeom[1],2)
        self.space.setSurfaceType(self.wheelsgeom[3],2)
        
        self.maxVelocity = 65
        self.maxSpeed=50
        self.accForce=500
        
        # register actions and tasks
        axis=[1,3]
        axis2=[1,3,0,2]
        base.accept('w', self.Accel,[self.maxVelocity, 40,axis])
        base.accept('w-up', self.Accel,[0, 15,axis2])
        base.accept('s', self.Accel,[-25, 40,axis])
        base.accept('s-up', self.Accel,[0, 15,axis2])
        base.accept('space', self.Accel, [0, 200,axis2])
        base.accept('space-up', self.Accel, [0, 15,axis2])
        base.accept('shift', self.Accel, [0, 50,axis2])
        base.accept('shift-up', self.Accel, [0, 15,axis2])
        base.accept('d', self.Turn,[True,0.01])       
        base.accept('a', self.Turn,[True,-0.01])       
        base.accept('d-up', self.Turn,[False,0.01])       
        base.accept('a-up', self.Turn,[False,-0.01])
        taskMgr.add(self.TurnTask,"Rule Car")
        taskMgr.add(self.JetTask,"Jet Task")
        taskMgr.doMethodLater(0.5,self.checkRotation, "checkRotation")
        #Setup the camera basis
        self.camPosNode = self.box.attachNewNode('camPosNode')
        self.camPosNode.setPos(0,6,-2)
        self.camLookatNode = self.box.attachNewNode('camLookatNode')
        self.camLookatNode.setPos(0,0,2)
        base.camLens.setFar(10000)
        
        #spedometer
        spdm = OnscreenImage(image = 'spdm.png', scale=0.25, pos = (1, 0, -0.6))
        spdm.setTransparency(TransparencyAttrib.MAlpha)
        self.pointer = OnscreenImage(image = 'spdm_pointer.png', scale=0.25, pos = (1, 0, -0.6))
        self.pointer.setTransparency(TransparencyAttrib.MAlpha)
        self.lastPos = Vec3(0,0,0)
    
    def addCamdist(self, v):
      self.camDistance += v
      print "new camdistance:", self.camDistance
    
    #acceleration function
    def Accel(self, aspect, force, axis):
        for i in [1,3,0,2]:
            self.joints[i].setParamFMax(1, 0)
        #We use two different methods for move forward and backward
        #Forward - "jet engine" - add force to the body of the car
        #Backward - angular engine - add angular speed to the wheels
        if aspect>0:
            self.acceleration=True
        else:
            self.acceleration=False
            for i in axis:
                #set angular engine speed
                self.joints[i].setParamVel(1,aspect*self.carOrientation)
                #and force to it
                self.joints[i].setParamFMax(1, force)
    #check car orientation, and change control according to it
    def checkRotation(self,task):
        oldO=self.carOrientation
        if abs(int(self.box.getR()))<90:
            self.carOrientation=1
        else:
            self.carOrientation=-1
        if oldO<>self.carOrientation:
            self.camPosNode.setZ(-self.camPosNode.getZ())
            for i in [1,3,0,2]:
                self.joints[i].setParamVel(1,-self.joints[i].getParamVel(1))
        return task.again
    #turn wheels - set variables
    def Turn(self,enabled,aspect):
        self.turn=enabled
        self.turnspeed=aspect
    #immediately, turn wheels here
    def TurnTask(self,task):
        #calculate angle
        if not self.turn:
            if self.turnangle>0:
                self.turnspeed=-0.01*self.carOrientation
            if self.turnangle<0:
                self.turnspeed=0.01*self.carOrientation
            if -0.01<self.turnangle<0.01:
                self.turnangle=0;
        self.turnangle=self.turnangle+self.turnspeed*self.carOrientation
        if self.turnangle>0.3:
            self.turnangle=0.3
        if self.turnangle<-0.3:
            self.turnangle=-0.3
        # and set angle to the front wheels
        self.joints[0].setParamHiStop(0, self.turnangle)
        self.joints[0].setParamLoStop(0, self.turnangle)
        self.joints[2].setParamHiStop(0, self.turnangle)
        self.joints[2].setParamLoStop(0, self.turnangle)
        # will fix wheel position a bit better
        for i in xrange(4):
          self.wheelsbody[i].setFiniteRotationAxis(self.joints[i].getAxis2())        
        return task.cont
    #task for jet engeene    
    def JetTask(self,task):
        if self.acceleration<>0:
            if self.maxSpeed>self.body.getLinearVel().length():
                self.body.addRelForce(0,self.accForce,0)
        return task.cont
    #sync our visible geometry with them physic bodyes
    def Sync(self):
        self.box.setPos(render, self.body.getPosition())
        self.box.setQuat(render,Quat(self.body.getQuaternion()))
        for i in range(4):
            self.wheels[i].setPos(render, self.wheelsbody[i].getPosition())
            self.wheels[i].setQuat(render,Quat(self.wheelsbody[i].getQuaternion()))
        # update the camera
        camVec = self.camPosNode.getPos(render) - self.body.getPosition()
        
        camDistance = Vec2(-5, 3)
        targetCamPos = self.body.getPosition() + camVec * camDistance.getX() + Vec3(0,0,camDistance.getY())
        camLookat = self.camLookatNode.getPos(render)
        dPos = targetCamPos - base.camera.getPos(render)
        dt = globalClock.getDt()
        base.camera.setPos(base.camera.getPos(render) + dPos * dt / .5)
        base.camera.lookAt(camLookat)
        # the speedometer pointer
        curPos = self.box.getPos(render)
        vel = (self.lastPos - curPos).length() * 6000 / self.maxVelocity
        self.lastPos = curPos
        dr=vel-self.pointer.getR()
        if dr>30:
            dr=30
        dr=dr*0.1
        self.pointer.setR(self.pointer.getR()+dr)

World()

run()
