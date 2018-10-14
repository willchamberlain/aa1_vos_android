



class VisualFeature:

    def __init__(self, target_id_, algorithm_, feature_id_, rot_, transl_ ):
        self.target_id = target_id_
        self.algorithm="boofcv square fiducial" 
        #  self.descriptor="330"
        self.feature_id=feature_id_
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
        target_id = "Pioneer3"  
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








