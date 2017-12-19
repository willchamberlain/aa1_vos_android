#!/usr/bin/env python

import sys
import rospy

from vos_aa1.srv import RobotId
from vos_aa1.srv import RobotIdRequest
from vos_aa1.srv import RobotIdResponse


from vos_aa1.robotId import RobotIdentifier





# the robot's own IDs for themselves, e.g. MAC addresses or something
# plus the assigned VOS ids
vos_robot_ids = []


def handle_robot_id_request(RobotId_req):
    print 'vos_server_robot_id: handling request(%s)'%RobotId_req.native_id
    for robot_id_ in vos_robot_ids : 
        if RobotId_req.native_id == robot_id_.native_id :            
            print 'vos_server_robot_id: %s already exists: robot id=%s' % ( robot_id_.native_id , robot_id_.vos_id )
            return RobotIdResponse(robot_id_.vos_id)
        print 'vos_server_robot_id: robot id=%s'%robot_id_.vos_id
    id_ = RobotIdentifier(RobotId_req.native_id)
    vos_robot_ids.append(id_)
    print 'vos_server_robot_id: assigned id (%s, %s) to request(%s)'%(id_.vos_id , id_.native_id, RobotId_req.native_id)
    return RobotIdResponse(id_.vos_id)

def start_node():
    rospy.init_node('vos_server_robot_id_manager')        
    rate = rospy.Rate(10) # 10hz
    first_iteration_ = True
    while not rospy.is_shutdown():
        if first_iteration_:
            rospy.loginfo('getting ready to manage robot ids')
            start_service()
            rospy.loginfo('ready to manage robot ids')
            first_iteration_ = False
        rate.sleep()
    
def start_service():  
    service_name_ = 'vos_server_robot_id'  
    s = rospy.Service(service_name_, RobotId, handle_robot_id_request)    
    rospy.loginfo('ready to manage robot ids: ' + 'service name=' + service_name_ + ': URI=' + rospy.get_node_uri() + ': namespace=' + rospy.get_namespace() + ': node name='+rospy.get_name())


if __name__ == "__main__":
    try:
        start_node()
        start_service()
    except rospy.ROSInterruptException:
        pass    
    

