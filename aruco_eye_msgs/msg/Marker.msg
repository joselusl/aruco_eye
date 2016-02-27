# Header. Needed for the frame_id
Header header

# Id of the visual marker
uint32 id

# Size of the visual marker if available
float32 size

# Points of the visual marker in the Image
aruco_eye_msgs/PointInImage[] pointsInImage

# Flag for the 3D reconstruction
bool is3dReconstructed

# Pose of the Visual Marker wrt Camera if calibration file is available
geometry_msgs/PoseWithCovariance pose

# Confidence in the detection. TODO
float64 confidence
