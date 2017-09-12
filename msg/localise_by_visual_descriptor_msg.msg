
string                              request_id      # id of the request to use in there_is_al_desc messages to differentiate from other requests from the same Vision OS client

string                              algorithm       # image processing algorithm to detect and extract the feature - e.g. Apriltags 36h11, BoofCV binary fiducial
string                              descriptor      # feature descriptor

geometry_msgs/Pose                  relation_to_base  # relation of the visual feature to some base frame: if not null, there_is_alg_desc.pose gives the pose of the base frame, not the visual feature, saving the client some transformation work

int32                               rate

string                              return_url      # return URL for the response

time                                valid_until     # Optional - maximum time to respond to this request in ms =  max( (time of receipt + run_period) , valid_until ): if rate is positive this defines a period to repeat for. (Server not guaranteed to respect this.?) Positive/future for repeat over several seconds. 0 or negative for once-off.
int32                               run_for_ms      # Optional - maximum time to respond to this request in ms =  max( (time of receipt + run_period) , valid_until ): if rate is positive this defines a period to repeat for. (Server not guaranteed to respect this.?) Positive/future for repeat over several seconds. 0 or negative for once-off.
int32                               repetitions     # Optional - taken to be 1 if no other durations are specified - the mminimum number of repititions: 0 or negative allows zero if run_for_ms is zero and valid_until is in the past of the receipt time.
