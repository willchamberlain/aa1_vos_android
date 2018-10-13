#!/usr/bin/python           

# This is server.py file

import sys
import socket               # Import socket module
import re



print "Starting"
print "------------------------------------------------"
test_message="HI : 970:  transl=[0.25543432624997575,0.21966393316100702,1.3709588212970767]  rot=[0.9996317594954752, 0.024130660901746405, 0.012411954416015796; 0.01986321504658553, -0.9623276093714678, 0.271166046785745; 0.018487782343846026, -0.2708196511441529, -0.9624525538737824 ] "
data = test_message
print data     
pattern_translation_block = r"transl=\[(.*)\]"  #  raw string:  to avoid having to double-escape some characters:  see https://docs.python.org/2/library/re.html
translation_block = re.search(pattern_translation_block, data)  #  https://docs.python.org/2/library/re.html
pattern_rotation_block = r"rot=\[(.*)\]"  #  raw string:  to avoid having to double-escape some characters:  see https://docs.python.org/2/library/re.html    
rotation_block = re.search(pattern_rotation_block, data)  #  https://docs.python.org/2/library/re.html
print translation_block.group(0)
print rotation_block.group(0)

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
        
        
        
