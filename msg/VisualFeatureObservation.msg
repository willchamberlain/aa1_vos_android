# A visual feature observed in the world at some time. AprilTags for now, but eventually SIFT, SURF, BRIEF, object-class, etc.
# Note: geometry_msgs/PoseStamped rather than geometry_msgs/Pose for VisualFeatureInWorld
# see https://github.com/RIVeR-Lab/apriltags_ros/blob/indigo-devel/apriltags_ros/msg/AprilTagDetection.msg
# TODO: make general with string parsing
string                              algorithm
int32                               id
geometry_msgs/PoseStamped           pose