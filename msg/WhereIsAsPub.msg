string                              algorithm       # image processing algorithm to detect and extract the feature - e.g. Apriltags 36h11, BoofCV binary fiducial
string                              descriptor      # feature descriptor
string                              request_id      # id of the request to use in there_is_al_desc messages to differentiate from other requests from the same Vision OS client
int32                               rate
geometry_msgs/Pose                  relation_to_base  # relation of the visual feature to some base frame: if not null, there_is_alg_desc.pose gives the pose of the base frame, not the visual feature, saving the client some transformation work
string                              return_url      # return URL for the response
