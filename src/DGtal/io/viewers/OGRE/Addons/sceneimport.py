#!BPY

""" Registration info for Blender menus:
Name: 'Scene importer (.scene)...'
Blender: 236
Group: 'Import'
Tip: 'Import an DGtal-Scene (.scene) file.'
"""

__author__ = "Anis Benyoub"
__version__ = "0.4.1 10/29/09"

__bpydoc__ = """\
This script imports DGtal-Scene into Blender.

Supported:<br>
    * Cubes, spheres, lines
    * Lights
    * materials (textures only)

Missing:<br>
    * Cameras
"""

# Copyright (c) 2005-2009 Anis Benyoub


import Blender
import glob
import os
import re
import xml.sax
import xml.sax.handler
import math

IMPORT_LOG_LEVEL = 1
IMPORT_SCALE_FACTOR = 1.0



def log(msg):
    if IMPORT_LOG_LEVEL >= 1: print msg

def vlog(msg):
    if IMPORT_LOG_LEVEL >= 2: print msg

def dlog(msg):
    if IMPORT_LOG_LEVEL >= 3: print msg


class Scene:
    def __init__(self):
    	self.scene = Blender.Scene.New('myScene')
    	self.scene.makeCurrent()
        self.cubes = []
        self.spheres = []
        self.manualObjects = []
        self.lights = []
        self.cameras = []         
        
class Cube:
    global IMPORT_SCALE_FACTOR
    def __init__(self,name, attrs,aScene):
		print("Got a Cube")	
		#Create a single cube.
		self.tempMesh = Blender.Mesh.Primitives.Cube(1)
		self.ob = Blender.Object.New("Mesh",name)
	 	self.ob.LocX=float(attrs["posx"])
	 	self.ob.LocY=float(attrs["posy"])
	 	self.ob.LocZ=float(attrs["posz"])
	 	scalex=float(attrs["scalex"])*100
	 	scaley=float(attrs["scaley"])*100
	 	scalez=float(attrs["scalez"])*100
		self.ob.setSize([scalex, scaley, scalez])
	 	self.ob.link(self.tempMesh)
	 	aScene.scene.link(self.ob)


class Sphere:
    global IMPORT_SCALE_FACTOR
    def __init__(self,name, attrs,aScene):
		print("Got a Sphere")	    
        #Create a single sphere.
		self.tempMesh = Blender.Mesh.Primitives.UVsphere(10,10,0.3)
		self.ob = Blender.Object.New("Mesh",name)
	 	self.ob.LocX=float(attrs["posx"])
	 	self.ob.LocY=float(attrs["posy"])
	 	self.ob.LocZ=float(attrs["posz"])
	 	scalex=float(attrs["scalex"])*400
	 	scaley=float(attrs["scaley"])*400
	 	scalez=float(attrs["scalez"])*400
		self.ob.setSize([scalex, scaley, scalez])
	 	self.ob.link(self.tempMesh)
	 	aScene.scene.link(self.ob)
	 	
	 	
class Light:
    global IMPORT_SCALE_FACTOR
    def __init__(self,name, attrs,aScene):
		if attrs["LightType"]=="Point" :
			self.lamp_obj = Blender.Object.New('Lamp')
			self.lamp_data = Blender.Lamp.New('Lamp')
			print("Got a Lamp")	    
			self.lamp_obj.link( self.lamp_data)
			aScene.scene.link(self.lamp_obj)
			self.lamp_obj.loc = int(attrs["posx"]), int(attrs["posy"]), int(attrs["posz"])# position at x,y,z
		elif attrs["LightType"]=="Spot" :
			self.lamp_obj = Blender.Object.New('Lamp')
			self.lamp_data = Blender.Lamp.New('Spot')
			print("Got a Spot")	    
			self.lamp_obj.link( self.lamp_data)
			aScene.scene.link(self.lamp_obj)
			# position object
			self.lamp_obj.loc = int(attrs["posx"]), int(attrs["posy"]), int(attrs["posz"])# position at x,y,z
        
class ManualObject:
    global IMPORT_SCALE_FACTOR
    def __init__(self,name, attrs):
		self.cube = bpy.data.objects["Cube"] 

        
class Camera:
    global IMPORT_SCALE_FACTOR
    def __init__(self,name, attrs, aScene):  
		# create object, obdata and link
		self.cam_obj = Blender.Object.New('Camera')
		self.cam_data = Blender.Camera.New('ortho')
		self.cam_obj.link(self.cam_data)
		aScene.scene.link(self.cam_obj)
		print("Got a Camera")	      
		# position object
		self.cam_obj.loc = (int(attrs["posx"]), int(attrs["posy"]), int(attrs["posz"]))
		# rotate camera to look towards origin
		self.cam_obj.rot = math.radians(90), 0, 0                        
    


class Material:
    def __init__(self, name):
        self.name = name
        self.texname = ""
        self.diffuse = (1.0, 1.0, 1.0, 1.0)
        self.ambient = (1.0, 1.0, 1.0, 1.0)
        self.specular = (0.0, 0.0, 0.0, 0.0)
        self.blenderimage = 0
        self.loading_failed = 0

    def getTexture(self):
        if self.blenderimage == 0 and not self.loading_failed:
            try:
                f = file(self.texname, 'r')
                f.close()
                self.blenderimage = Blender.Image.Load(self.texname)
            except IOError, (errno, strerror):
                errmsg = "Couldn't open %s #%s: %s" \
                        % (self.texname, errno, strerror)
                log(errmsg)
                self.loading_failed = 1;
                
        return self.blenderimage


class OgreSceneHandler(xml.sax.handler.ContentHandler):
    

    
    def __init__(self):
        self.scene = 0

    def startDocument(self):
        self.scene = Scene()
        
    def startElement(self, name, attrs):
        if name == 'Cube':
            self.scene.cubes.append(Cube(name,attrs,self.scene))
        elif name == 'Sphere':
            self.scene.spheres.append(Sphere(name,attrs,self.scene))
        elif name == 'ManualObject':
        	#print("Got a manual object")	
            self.scene.manualObjects.append(ManualObject(name,attrs))
        elif name == 'Light':
        	#print("Got a light")	
            self.scene.lights.append(Light(name,attrs,self.scene))
        elif name == 'Camera':
        	#print("Got a camera")	
            self.scene.cameras.append(Camera(name,attrs,self.scene))

def fileselection_callback(filename):
    log("Reading scene file %s..." % filename)
    
    dirname = Blender.sys.dirname(filename)
    basename = Blender.sys.basename(filename)
    
    
    parser = xml.sax.make_parser()   
    handler = OgreSceneHandler()
    parser.setContentHandler(handler)
    parser.parse(open(filename))
   
    log("import completed.")
    
    
    Blender.Redraw()
    
    
try:
	Blender.Window.FileSelector(fileselection_callback, "Import Scene", "*.scene")
except: 
	print("Something went wrong")	
