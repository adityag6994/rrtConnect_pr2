#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/myplugin')
# try:
env=Environment()
env.Load('scenes/hw3.env.xml')
MyNewModule = RaveCreateModule(env,'RRTModule')
print MyNewModule.SendCommand('help')
# finally:
print 'hi'
	# RaveDestroy()
