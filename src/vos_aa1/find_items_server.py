#!/usr/bin/env python

import rospy
from vos_aa1.srv import *

def on_find_items(req):
      return FindItemsResponse([1, 2, 3])

if __name__ == '__main__':
    node_name = "knowledge_representation"
    rospy.init_node(node_name, anonymous=True)
    service = rospy.Service("kr/find_items", FindItems, on_find_items)

    rospy.spin()