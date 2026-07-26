# One raw or tracked danger-source observation from perception.
# Standard alternatives are not enough because this carries competition-specific
# class, confidence, tracking, and rejection status semantics.

std_msgs/Header header

# Estimated danger-source center in header.frame_id, meters.
geometry_msgs/Point position

# 0.0 to 1.0.
float32 confidence

# Example: danger_red_sphere.
string class_name

# Perception-local tracking ID. 0 means untracked.
uint32 track_id

# False means the candidate is published for diagnostics but must not be used by
# mission/result modules.
bool is_valid

# Examples: ok, tracked, tf_unavailable, insufficient_valid_depth.
string status
