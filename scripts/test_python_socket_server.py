#!/usr/bin/python           

# This is server.py file

import sys
import socket               # Import socket module
import re

import numpy as np

import tf
from tf import transformations

from visual_feature import VisualFeature
from vos_target import VOSTarget



print "Starting"
print "------------------------------------------------"
test_message="HI : 970:  transl=[0.25543432624997575,0.21966393316100702,1.3709588212970767]  rot=[0.9996317594954752, 0.024130660901746405, 0.012411954416015796; 0.01986321504658553, -0.9623276093714678, 0.271166046785745; 0.018487782343846026, -0.2708196511441529, -0.9624525538737824 ] "
test_message="HI : 1170:  transl=[-0.39539658600098126,0.12503715015778222,0.9852511701550373]  rot=[-0.024042460921678832, -0.9994326987736851, 0.02358475555901626; -0.9994398292296162, 0.023479865647488052, -0.023847927764319866; 0.02328063191379955, -0.02414490693966213, -0.9994373595411417 ] "
test_message="HI : feature_id=[1170]  transl=[-0.39539658600098126,0.12503715015778222,0.9852511701550373]  rot=[-0.024042460921678832, -0.9994326987736851, 0.02358475555901626; -0.9994398292296162, 0.023479865647488052, -0.023847927764319866; 0.02328063191379955, -0.02414490693966213, -0.9994373595411417 ] "
test_message="HI : feature_id=[1170]  transl=[-0.42736654300831506,0.12622572055441045,0.9806823299211838]  rot=[0.007784502348537559, -0.9991051602160963, 0.041572591364435973; -0.980966809944904, -0.01569634465831915, -0.19354002829101308; 0.19401937869631608, -0.03927471952714641, -0.9802111900485321 ] "

data = test_message
print data     
pattern_feature_id = r"feature_id=\[(.*)\]  transl="
feature_id = re.search(pattern_feature_id, data)
pattern_translation_block = r"transl=\[(.*)\]  rot="  #  raw string:  to avoid having to double-escape some characters:  see https://docs.python.org/2/library/re.html
translation_block = re.search(pattern_translation_block, data)  #  https://docs.python.org/2/library/re.html
pattern_rotation_block = r"rot=\[(.*)\]"  #  raw string:  to avoid having to double-escape some characters:  see https://docs.python.org/2/library/re.html    
rotation_block = re.search(pattern_rotation_block, data)  #  https://docs.python.org/2/library/re.html


print "------------------------------------------------"
print 'feature_id.group(1)  :'
print feature_id.group(1)
print "------------------------------------------------"
print 'translation_block.group(1)  :'
print translation_block.group(1)
print "------------------------------------------------"
print 'rotation_block.group(1)  :'
print rotation_block.group(1)
print "------------------------------------------------"
pattern_numbers = r"(-?[0-9]+(\..[0-9]*))"  # zero or one '-' , one or more number , (  zero or one period , zero or one number  )
pattern_numbers = r"(-?[0-9]+\.[0-9]+)"  # zero or one '-' , one or more number , one period , one or more number
pattern_integer = r"(-?[0-9]+)"  # zero or one '-' , one or more number
numbers_in_translation_block = re.findall(pattern_numbers,translation_block.group(1))
print 'numbers_in_translation_block  :'
print numbers_in_translation_block
transl_nums = numbers_in_translation_block
 

print "------------------------------------------------"
#  flip to ROS coordinate system

try_  = np.array((
    (0. , 0. , 1. ),
    (1. , 0. , 0. ),
    (0. , 1. , 0. )
    ), dtype=np.float64)
try_inverse = try_.transpose()
print try_
print try_inverse
print try_.dot(try_inverse)    
print "----"
r = np.array((
    ( -0.017206208582719595, -0.9984396381082828, -0.05312471590897444 ),
    ( -0.9388669558110284, 0.034406250870812505, -0.3425566364663618), 
    (  0.3438499464480697, 0.0439829393653495, -0.9379939847208209)
    ), dtype=np.float64)

t = np.array((
    (  -0.41573823276137206),
    (   0.12028135881899521),
    (   1.0091068950063067)
    ), dtype=np.float64)
    
numbers_in_rotation_block = re.findall(pattern_numbers,rotation_block.group(1))
rot_nums = numbers_in_rotation_block
r = np.array((
    ( rot_nums[0], rot_nums[1], rot_nums[2] ),
    ( rot_nums[3], rot_nums[4], rot_nums[5]), 
    ( rot_nums[6], rot_nums[7], rot_nums[8])
    ), dtype=np.float64)    

a_ = try_.dot(r.dot(try_inverse))

rotx_neg90 = np.array((
    (    1.0000,    0.0000,    0.0000 ),
    (    0.0000,    0.0000,    1.0000 ),
    (    0.0000,   -1.0000,    0.0000 )
    ), dtype=np.float64)
camera_to_tag_rotation = rotx_neg90.dot(a_)    
print r
print t
print a_
print rotx_neg90
print "----"
print "camera_to_tag_rotation:"
print camera_to_tag_rotation
print "----"
print "camera_to_tag_translation:"
transl_vec = np.array(( 
    ( transl_nums[0] ) , ( transl_nums[1] ) , ( transl_nums[2] )
    ), dtype=np.float64)
camera_to_tag_translation = try_.dot(transl_vec)
print camera_to_tag_translation
print "----"

print "------------------------------------------------"

#R = tf.transformations.quaternion_matrix(quat_)
#R_rect = np.array((
#    (R[0,2] , -1.0*R[0,1] , R[0,0], 0.0),
#    (R[1,2] , -1.0*R[1,1] , R[1,0], 0.0),
#    (R[2,2] , -1.0*R[2,1] , R[2,0], 0.0),
#    ( 0.0   , 0.0         , 0.0   , 1.0) 
#    ), dtype=np.float64)
#quat_rect = tf.transformations.quaternion_from_matrix(R_rect)

R_rect = np.array((
    (rot_nums[0] , rot_nums[1] , rot_nums[2], 0.0),
    (rot_nums[3] , rot_nums[4] , rot_nums[5], 0.0),
    (rot_nums[6] , rot_nums[7] , rot_nums[8], 0.0),
    ( 0.0   , 0.0         , 0.0   , 1.0) 
    ), dtype=np.float64)
print 'R_rect:'    
print R_rect
quat_rect = tf.transformations.quaternion_from_matrix(R_rect)
print 'quat_rect:'
print quat_rect

print "------------------------------------------------"
print "tested regex"
print "------------------------------------------------"


s = socket.socket()         # Create a socket object
host = socket.gethostname() # Get local machine name
print host
port = 5000                # Reserve a port for your service.
print socket.getaddrinfo(host,port)
s.bind(('192.168.43.252', port))        # Bind to the port
print "bound"
print "------------------------------------------------"

print "Set up robot/targets"
vos_target_list = [] 
VOSTarget

print "------------"

print "Set up visual features for robots"
visual_feature_list = []

rot = np.array((
    ( 0.0000,   -1.0000,    0.0000 ),
    ( 1.0000,    0.0000,    0.0000 ),
    ( 0.0000,    0.0000,    1.0000 )
    ), dtype=np.float64)
transl = np.array(( ( 0.1 , 0.15 , 1.0 ) ), dtype=np.float64)        
vf1170 = VisualFeature( "boofcv square fiducial" , 1170, rot, transl )
visual_feature_list.append( vf1170 )

rot = np.array((
    ( 0.7071,    0.7071,    0.0000 ),
    (-0.7071,    0.7071,    0.0000 ),
    ( 0.0000,    0.0000,    1.0000 )
    ), dtype=np.float64)
transl = np.array(( ( 0.2 , 0.25 , 1.2 ) ), dtype=np.float64)        
vf1177 = VisualFeature( "boofcv square fiducial" , 1177, rot, transl )
visual_feature_list.append( vf1177 )  

transl = np.array(( ( 0.2 , 0.25 , 1.23 ) ), dtype=np.float64)        
vf1178 = VisualFeature( "boofcv square fiducial" , 1178, rot, transl )
visual_feature_list.append( vf1178 )  

transl = np.array(( ( 0.2 , 0.25 , 1.25 ) ), dtype=np.float64)        
vf1179 = VisualFeature( "boofcv square fiducial" , 1179, rot, transl )
visual_feature_list.append( vf1179 )

value = 1178
x = []
for x in visual_feature_list:
    if x.feature_id == value:
        print "I found : "
        print value
        break
else:
    x = None
print x    
if x != None :
    print "Found 1178!"
    print x.feature_id
    print x.transl
else :
    print "Not found 1178!"

print "---------------------"

x = []
for x in visual_feature_list:
    print x.feature_id
    print x.feature_id == 1178
x = []
next((x for x in visual_feature_list if x.feature_id == 1178), None)
if x :
    print "Found 1178!"
    print x.feature_id
    print x.transl
else :
    print "Not found 1178!"
    
x = []
next((x for x in visual_feature_list if x.feature_id == 1187), None)
if x :
    print "Found 1187!"
    print x.feature_id
    print x.transl
else :
    print "Not found 1187!"

print "------------------------------------------------"


s.listen(5)                 # Now wait for client connection.
print "listening"
print "------------------------------------------------"
while True:
    c, addr = s.accept()     # Establish connection with client.
    print 'Got connection from', addr
    while 1:
#        try:
        data = c.recv(1024)
        if not data:
            break
        print "------------------------------------------------"
        print "Received:"
        print data     
        try:    
            print "------------------------------------------------"
            encoding_str = 'utf-8'
            print "Trying to data.decode('%s')"%(encoding_str)
#        data = data.decode(encoding='utf-8')
            stringdata = data.decode(encoding_str)
            print "decoded to: "
            print stringdata
        except:
            print "Error decoding the data: "
            print sys.exc_info()[0]
           
            
        #  Operate on every single request    
        try:
            feature_id_block = re.search(pattern_feature_id, data)
            translation_block = re.search(pattern_translation_block, data)  #  https://docs.python.org/2/library/re.html
            rotation_block = re.search(pattern_rotation_block, data)  #  https://docs.python.org/2/library/re.html
            print "------------------------------------------------"
            feature_id_nums = re.findall(pattern_integer,feature_id_block.group(1))
            feature_id = feature_id_nums[0]
            
            numbers_in_translation_block = re.findall(pattern_numbers,translation_block.group(1))
            transl_nums = numbers_in_translation_block
            transl_vec = np.array(( 
                ( transl_nums[0] ) , ( transl_nums[1] ) , ( transl_nums[2] )
                ), dtype=np.float64)
            camera_to_tag_translation = try_.dot(transl_vec)
            
            numbers_in_rotation_block = re.findall(pattern_numbers,rotation_block.group(1))
            rot_nums = numbers_in_rotation_block
            r = np.array((
                ( rot_nums[0], rot_nums[1], rot_nums[2] ),
                ( rot_nums[3], rot_nums[4], rot_nums[5]), 
                ( rot_nums[6], rot_nums[7], rot_nums[8])
                ), dtype=np.float64)    
            a_ = try_.dot(r.dot(try_inverse))
            camera_to_tag_rotation = rotx_neg90.dot(a_)   
            print "------------------------------------------------"
            
            print "feature_id"
            print feature_id
            print "---"
            print 'camera_to_tag_translation'
            print camera_to_tag_translation
            print "---"
            print 'camera_to_tag_rotation'
            print camera_to_tag_rotation
            
            
            
            
        except:
            print "Error parsing the data: "
            print sys.exc_info()[0]
        
            
        try:
            pattern_translation_block = r"transl=\[(.*)\]  rot="  #  raw string:  to avoid having to double-escape some characters:  see https://docs.python.org/2/library/re.html
            translation_block = re.search(pattern_translation_block, stringdata)  #  https://docs.python.org/2/library/re.html
            if translation_block :
                print "Matched translation block:"
                print translation_block.group(1)
            else:
                print "Did NOT match rotation block"
            pattern_rotation_block = r"rot=\[(.*)\]"  #  raw string:  to avoid having to double-escape some characters:  see https://docs.python.org/2/library/re.html    
            rotation_block = re.search(pattern_rotation_block, stringdata)  #  https://docs.python.org/2/library/re.html
            if rotation_block :
                print "Matched rotation block:"
                print rotation_block.group(1)
            else:
                print "Did NOT match rotation block"
        except:
            print "Error parsing the decoded data with regex: "
            print sys.exc_info()[0]
        
        print "------------------------------------------------"
#        except:   
#            print("Unexpected error: ", sys.exc_info()[0])
    c.send('Thank you for connecting')
    c.close()                # Close the connection
        
        
        
