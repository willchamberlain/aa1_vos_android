#!/usr/bin/env python

import rospy, os, time
from vos_aa1.srv import *

if __name__ == '__main__':
    node_name = "knowledge_representation_test"
    rospy.init_node(node_name, anonymous=True)

    service_name = "kr/find_items"
    rospy.wait_for_service(service_name)
    findItemsProxy  = rospy.ServiceProxy(service_name,  FindItems)

    nodeIDs = findItemsProxy.call("Node", "cvc")

    print(nodeIDs)