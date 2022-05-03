#!/usr/bin/env python
# -*- coding: utf-8 -*-

########## INIT ####################################################################################

##### Future First #####
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

##### Imports #####
from math import pi , sqrt
import time
now = time.time
import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg


########## HELPER FUNCTIONS ########################################################################

def get_ROS_ntime_secs():
    """ The the current time in decimal seconds down to decimal nanosecond """
    now = rospy.get_rostime()
    rtnNum = now.secs + now.nsecs / 1e10 
    return rtnNum


def pose_msg( self , posXYZ , ornXYZW , baseFrame , trgtFrame , pTime ):
    """ Send a stamped transform with the given data """
    # 1. Create message
    xform = geometry_msgs.msg.TransformStamped()
    # 2. Compose message
    # ~ Transform Header ~
    xform.header.stamp    = pTime
    xform.header.frame_id = baseFrame
    xform.child_frame_id  = trgtFrame
    # ~ Position ~
    xform.transform.translation.x = posXYZ[0]
    xform.transform.translation.y = posXYZ[1]
    xform.transform.translation.z = posXYZ[2]
    # ~ Orientation ~
    xform.transform.rotation.x = ornXYZW[0]
    xform.transform.rotation.y = ornXYZW[1]
    xform.transform.rotation.z = ornXYZW[2]
    xform.transform.rotation.w = ornXYZW[3]
    # 3. Return message
    return xform


def unpack_ROS_xform( xform ):
    """ Unpack the ROS transform message into position and orientation """
    posn = [ xform.transform.translation.x , xform.transform.translation.y , xform.transform.translation.z ] 
    ornt = [ xform.transform.rotation.x    , xform.transform.rotation.y    , xform.transform.rotation.z    , xform.transform.rotation.w ]
    return posn , ornt



########## CLASSES #################################################################################

class BasicNode:
    """ Base class for ROS1 Nodes """

    def __init__( self, nodeName = None, updateHz = 300 ):
        """ Set the update rate for this node """
        # 1. Start the node
        if nodeName is None:
            self.name = self.__class__.__name__
        else:
            self.name = nodeName
        rospy.init_node( nodeName ) 
        self.bgn = now()
        # 2. Set up refresh rate
        self.updateHz = updateHz
        self.idle     = rospy.Rate( self.heartBeatHz ) # Best effort to maintain 'heartBeatHz' , URL: http://wiki.ros.org/rospy/Overview/Time        
        rospy.logdebug( str( self.name ) + ": begun at " + str( self.bgn ) )

    def update( self ):
        """ Run once every iteration by the `run` function """
        print( "THIS FUNCTION TO BE OVERRIDDEN" )
        pass

    def cleanup( self ):
        """ Run once after the node exits """
        print( "THIS FUNCTION TO BE OVERRIDDEN" )
        pass

    def obit( self ):
        """ Report the end of the node """
        self.end = now()
        rospy.logdebug( str( self.name ) + ": Shut down after " + str( self.end-self.bgn ) + " seconds!" )

    def run( self ):
    # 1. While ROS is running
        while not rospy.is_shutdown():
            try:
                self.update()
            except Exception as err: 
                rospy.logerr( self.name + ": had an error, " + str( err ) )

            # N-1: Wait until the node is supposed to fire next
            self.idle.sleep() 
        # N. Post-shutdown activities
        else:
            self.cleanup()
            self.obit()
    

# ___ END CLASS ____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================



# ___ End Spare __________________________________________________