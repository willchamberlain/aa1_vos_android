# A visual feature observed in the world at some time. AprilTags for now, but eventually SIFT, SURF, BRIEF, object-class, etc.
# Note: geometry_msgs/PoseStamped rather than geometry_msgs/Pose for VisualFeatureInWorld
# see https://github.com/RIVeR-Lab/apriltags_ros/blob/indigo-devel/apriltags_ros/msg/AprilTagDetection.msg
# TODO: make general with string parsing
string                      algorithm       # image processing algorithm to detect and extract the feature - e.g. Apriltags 36h11, BoofCV binary fiducial
string                      descriptor      # feature descriptor and/or parameters e.g. BoofCV square binary fiducial --> grid|width|id  4|257mm|557
int32                       id              # feature descriptor/id - tag number for AprilTags
geometry_msgs/PoseStamped   pose            # pose in camera coordinate frame
time                        time            # time feature observed
