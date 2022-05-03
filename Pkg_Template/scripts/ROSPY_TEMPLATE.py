#!/usr/bin/env python
# -*- coding: utf-8 -*-

########## INIT ####################################################################################

##### Future First #####
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

"""
James Watson , Template Version: 2020-05-18
Built on Gedit for Python 2.7

Dependencies: numpy , rospy
"""


"""  
~~~ Developmnent Plan ~~~
[ ] Find-Replace `CLASSNAME`
[ ] NEXT_STEP
"""

# === Init Environment =====================================================================================================================
# ~~~ Prepare Paths ~~~
import sys, os.path
SOURCEDIR = os.path.dirname( os.path.abspath( __file__ ) ) # URL, dir containing source file: http://stackoverflow.com/a/7783326
PARENTDIR = os.path.dirname( SOURCEDIR )

# ~~~ Imports ~~~
# ~~ Standard ~~
from math import pi , sqrt , sin , cos
# ~~ Special ~~
import numpy as np
import rospy , rospkg
# ~~ Local ~~
import rospy_helpers
from rospy_helpers import BasicROSNode
rospy_helpers.install_constants()
# ~~ Messages ~~

# ___ End Init _________________________________________________________________________________________________________


# === Main Application =================================================================================================

# ~~ Program Constants ~~


# == Program Classes ==

class CLASSNAME( BasicROSNode ):
    """ A_ONE_LINE_DESCRIPTION_OF_NODE_PURPOSE """

    def __init__( self , name = "CLASSNAME" , rate = 300 ):
        """ A_ONE_LINE_DESCRIPTION_OF_INIT """
        super( CLASSNAME , self ).__init__( nodeName=name , refreshRate=rate )
        
        # ~~~ 3. Subscribers and Listeners ~~~ 
        # rospy.Subscriber( "TOPIC_NAME" , MSG_TYPE , CALLBACK_FUNC )
        
        # ~~~  4. Publishers ~~~ 
        # self.pub = rospy.Publisher( "TOPIC_NAME" , MSG_TYPE , queue_size = 10 )
             
        
    def run( self ):
        """ A_ONE_LINE_DESCRIPTION_OF_RUNTIME_ACTIVITY """
        
        # 0. While ROS is running
        while ( not rospy.is_shutdown() ):
            
            # 1. FIXME: THINGS TO DO WHILE THE NODE IS RUNNING
            
            # N-1: Wait until the node is supposed to fire next
            self.idle.sleep()        
        
        # N. Post-shutdown activities
        else:
            self.obit() # Post-run uptime report
        

# __ End Class __


# ~~~ Start Node ~~~

if __name__ == "__main__":
    termArgs = sys.argv[1:] # Terminal arguments , if they exist

    try:
        obj = CLASSNAME()
        obj.run()    
    except rospy.ROSInterruptException:
        obj.obit()

    
# ___ End Main _________________________________________________________________________________________________________


# === Spare Parts ======================================================================================================



# ___ End Spare ________________________________________________________________________________________________________ ____________________________________________________________________________________________________________________________
