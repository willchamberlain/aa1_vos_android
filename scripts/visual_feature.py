
import numpy as np

class VisualFeature:

    def __init__(self, algorithm_, feature_id_, rot_, transl_ ):
        self.algorithm="boofcv square fiducial" 
        #  self.descriptor="330"
        self.feature_id=330
        #  xyz=" 0.000 -0.040  0.615"
        self.rot = rot_
        #  np.array((
        #      (  (  ,  ,  ) ),
        #      (  (  ,  ,  ) ),
        #      (  (  ,  ,  ) )
        #      ), dtype=np.float64)
        self.transl = transl_
        #  np.array((
        #      (  0.000 ),
        #      ( -0.040 ),
        #      (  0.615 )
        #      ), dtype=np.float64)
        
    def default_VisualFeature():    
        algorithm="boofcv square fiducial" 
        descriptor="330"
        feature_id=330
        #  xyz=" 0.000 -0.040  0.615"
        rot = np.array((
            (  ( 1. , 0. , 0. ) ),
            (  ( 0. , 1. , 0. ) ),
            (  ( 0. , 0. , 1. ) )
            ), dtype=np.float64)
        transl = np.array((
            (  0.000 ),
            ( -0.040 ),
            (  0.615 )
            ), dtype=np.float64)
        return VisualFeature( algorithm, feature_id, rot, transl )








