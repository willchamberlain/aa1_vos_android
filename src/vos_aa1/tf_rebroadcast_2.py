#!/usr/bin/env python

#  https://github.com/andreasBihlmaier/ahbros/blob/master/scripts/tf_alias_node.py

import roslib
import rospy
import sys
import argparse
import tf
import numpy
from tf.transformations import *



def main(args):
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=10, help='Frequency TFs are republished (default: 10 Hz)')
  parser.add_argument('old_from_tf', help='Read  Base/from/parent  Frame')
  parser.add_argument('old_to_tf', help='Read  Target/to/child Frame')
  parser.add_argument('new_from_tf', help='Published  Base/from/parent  Frame')
  parser.add_argument('new_to_tf', help='Published  Target/to/child  Frame')
  args = parser.parse_args(rospy.myargv()[1:])
  new_from_tf, new_to_tf, old_from_tf, old_to_tf = args.new_from_tf, args.new_to_tf, args.old_from_tf, args.old_to_tf
  print 'new_from_tf=%s , \ntarget_to_tf=%s , \nsource_from_tf=%s , \nsource_to_tf=%s'%(new_from_tf, new_to_tf, old_from_tf, old_to_tf)
  print 'old_from_tf=%s'%(old_from_tf)

  rospy.init_node('tf_alias_node', anonymous=True)
  tf_listener = tf.TransformListener()
  tf_broadcaster = tf.TransformBroadcaster()
  tf_listener.waitForTransform(old_from_tf, old_to_tf, rospy.Time(), rospy.Duration(4.0))

  rospy.loginfo('Spinning')
  r = rospy.Rate(args.freq)
  while not rospy.is_shutdown():
    print ' -- tf_listener.lookupTransform - start -- '
    position, quaternion = tf_listener.lookupTransform(old_from_tf, old_to_tf, rospy.Time())
    print ' -- tf_listener.lookupTransform - finished -- '
    #print('Publishing %s -> %s (= %s -> %s): position=%s quaternion=%s' % (new_from_tf, new_to_tf, old_from_tf, old_to_tf, position, quaternion))
    print ' -- tf_broadcaster.sendTransform - start -- '
    tf_broadcaster.sendTransform(position, quaternion, rospy.get_rostime(), new_to_tf, new_from_tf)
    print ' -- tf_broadcaster.sendTransform - finished -- '
    r.sleep()


if __name__ == '__main__':
  main(sys.argv)

#   tf_rebroadcast_2.py new_from_tf=laser new_to_tf=laser old_from_tf=Pioneer_p3dx old_to_tf=base_link 
#   tf_rebroadcast_2.py Pioneer_p3dx laser  base_link laser   