# A visual feature - AprilTags for now, but eventually SIFT, SURF, BRIEF, object-class, etc - located in the world - used to represent fixed points (points that do not move during the duration of the lifetime of the running system).
# Note: geometry_msgs/Pose rather than geometry_msgs/PoseStamped for VisualFeatureObservation
# see https://github.com/RIVeR-Lab/apriltags_ros/blob/indigo-devel/apriltags_ros/msg/AprilTagDetection.msg
# TODO: make general with string parsing
string                              algorithm       # image processing algorithm to detect and extract the feature - e.g. Apriltags 36h11
int32                               id              # feature descriptor/id - tag number for AprilTags
geometry_msgs/Pose                  pose            # pose in world coordinates - coordinate frame follows the robot coordinate frame convention, with the x axis along the surface normal - TODO work out what that would mean for e.g. intersection-of-occlusions features