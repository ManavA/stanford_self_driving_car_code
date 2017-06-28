#!/usr/bin/env python

import roslib
roslib.load_manifest('drivestatus')
import rospy
from drivestatus.msg import DriveStatus
from drivestatus.srv import *

class StatusCenter:

  def __init__(self):
    self.pub = rospy.Publisher('drivestatus', DriveStatus)
    rospy.Service('drivestatusrequest', ChangeDriveStatus, self.requestHandler)
    self.status = DriveStatus()
    rospy.init_node('statuscenter')

  def loop(self):
    while not rospy.is_shutdown():
      self.status.header.stamp = rospy.Time.now()
      center.pub.publish(self.status)
      rospy.sleep(0.5)

  def requestHandler(self, req):

    if self._isValid(req):
      rospy.loginfo("Request for status " + StatusCenter.statusToString(req.status) + " success")
      self.status.status = req.status
      self.status.header.stamp = rospy.Time.now()
      center.pub.publish(self.status)
      return ()
    else:
      rospy.loginfo("Request for status " + StatusCenter.statusToString(req.status) + " failed")
      return None
      

  def _isValid(self, req):
    '''
    Validate request for a change in status
    '''
    if req.status < DriveStatus.ERROR or req.status > DriveStatus.RUN:
      return False
    return True

  @staticmethod
  def statusToString(status):
    return {
        -2 : 'ERROR',
        -1 : 'ESTOP',
        0 : 'OFF',
        1 : 'STOP',
        2 : 'PAUSE',
        3 : 'RUN',
        }.get(status, 'INVALID')

if __name__ == '__main__':

  center = StatusCenter()
  center.status.status = DriveStatus.OFF

  try:
    center.loop()
  except rospy.ROSInterruptException:
    pass
