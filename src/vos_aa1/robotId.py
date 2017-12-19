
import random
import string



class RobotIdentifier:   
    
    
    def __init__(self, native_id_):
        self.native_id = native_id_
        self.vos_id = ''
        self.assign_vos_id(self.native_id)
    
    
    
    def assign_vos_id(self, native_id_):
        random.seed()
        str_ = ''
        for i_ in range(1,2):
            str_ = str_\
                + random.choice(string.ascii_uppercase)\
                + random.choice(string.ascii_uppercase)\
                + random.choice(string.ascii_uppercase)\
                + '_'\
                + str(random.randint(0,9))\
                + str(random.randint(0,9))\
                + str(random.randint(0,9))
        self.vos_id = str_
                
        
                
    def print_vos_id(self):
        print self.vos_id                
                
