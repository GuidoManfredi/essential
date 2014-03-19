; Auto-generated. Do not edit!


(cl:in-package segment_plans_objects-srv)


;//! \htmlinclude PlantopSegmentation-request.msg.html

(cl:defclass <PlantopSegmentation-request> (roslisp-msg-protocol:ros-message)
  ((num_plans_requested
    :reader num_plans_requested
    :initarg :num_plans_requested
    :type cl:integer
    :initform 0))
)

(cl:defclass PlantopSegmentation-request (<PlantopSegmentation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlantopSegmentation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlantopSegmentation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name segment_plans_objects-srv:<PlantopSegmentation-request> is deprecated: use segment_plans_objects-srv:PlantopSegmentation-request instead.")))

(cl:ensure-generic-function 'num_plans_requested-val :lambda-list '(m))
(cl:defmethod num_plans_requested-val ((m <PlantopSegmentation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segment_plans_objects-srv:num_plans_requested-val is deprecated.  Use segment_plans_objects-srv:num_plans_requested instead.")
  (num_plans_requested m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlantopSegmentation-request>) ostream)
  "Serializes a message object of type '<PlantopSegmentation-request>"
  (cl:let* ((signed (cl:slot-value msg 'num_plans_requested)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlantopSegmentation-request>) istream)
  "Deserializes a message object of type '<PlantopSegmentation-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_plans_requested) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlantopSegmentation-request>)))
  "Returns string type for a service object of type '<PlantopSegmentation-request>"
  "segment_plans_objects/PlantopSegmentationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlantopSegmentation-request)))
  "Returns string type for a service object of type 'PlantopSegmentation-request"
  "segment_plans_objects/PlantopSegmentationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlantopSegmentation-request>)))
  "Returns md5sum for a message object of type '<PlantopSegmentation-request>"
  "1200a1dffca987703a9988258c174cc5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlantopSegmentation-request)))
  "Returns md5sum for a message object of type 'PlantopSegmentation-request"
  "1200a1dffca987703a9988258c174cc5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlantopSegmentation-request>)))
  "Returns full string definition for message of type '<PlantopSegmentation-request>"
  (cl:format cl:nil "int64 num_plans_requested~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlantopSegmentation-request)))
  "Returns full string definition for message of type 'PlantopSegmentation-request"
  (cl:format cl:nil "int64 num_plans_requested~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlantopSegmentation-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlantopSegmentation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlantopSegmentation-request
    (cl:cons ':num_plans_requested (num_plans_requested msg))
))
;//! \htmlinclude PlantopSegmentation-response.msg.html

(cl:defclass <PlantopSegmentation-response> (roslisp-msg-protocol:ros-message)
  ((table
    :reader table
    :initarg :table
    :type object_recognition_msgs-msg:Table
    :initform (cl:make-instance 'object_recognition_msgs-msg:Table))
   (clusters
    :reader clusters
    :initarg :clusters
    :type (cl:vector sensor_msgs-msg:PointCloud2)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:PointCloud2 :initial-element (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
   (cluster_images
    :reader cluster_images
    :initarg :cluster_images
    :type (cl:vector sensor_msgs-msg:Image)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:Image :initial-element (cl:make-instance 'sensor_msgs-msg:Image)))
   (result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass PlantopSegmentation-response (<PlantopSegmentation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlantopSegmentation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlantopSegmentation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name segment_plans_objects-srv:<PlantopSegmentation-response> is deprecated: use segment_plans_objects-srv:PlantopSegmentation-response instead.")))

(cl:ensure-generic-function 'table-val :lambda-list '(m))
(cl:defmethod table-val ((m <PlantopSegmentation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segment_plans_objects-srv:table-val is deprecated.  Use segment_plans_objects-srv:table instead.")
  (table m))

(cl:ensure-generic-function 'clusters-val :lambda-list '(m))
(cl:defmethod clusters-val ((m <PlantopSegmentation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segment_plans_objects-srv:clusters-val is deprecated.  Use segment_plans_objects-srv:clusters instead.")
  (clusters m))

(cl:ensure-generic-function 'cluster_images-val :lambda-list '(m))
(cl:defmethod cluster_images-val ((m <PlantopSegmentation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segment_plans_objects-srv:cluster_images-val is deprecated.  Use segment_plans_objects-srv:cluster_images instead.")
  (cluster_images m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <PlantopSegmentation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segment_plans_objects-srv:result-val is deprecated.  Use segment_plans_objects-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<PlantopSegmentation-response>)))
    "Constants for message type '<PlantopSegmentation-response>"
  '((:NO_CLOUD_RECEIVED . 1)
    (:NO_TABLE . 2)
    (:OTHER_ERROR . 3)
    (:SUCCESS . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'PlantopSegmentation-response)))
    "Constants for message type 'PlantopSegmentation-response"
  '((:NO_CLOUD_RECEIVED . 1)
    (:NO_TABLE . 2)
    (:OTHER_ERROR . 3)
    (:SUCCESS . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlantopSegmentation-response>) ostream)
  "Serializes a message object of type '<PlantopSegmentation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'table) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'clusters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'clusters))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cluster_images))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cluster_images))
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlantopSegmentation-response>) istream)
  "Deserializes a message object of type '<PlantopSegmentation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'table) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'clusters) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'clusters)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:PointCloud2))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cluster_images) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cluster_images)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:Image))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlantopSegmentation-response>)))
  "Returns string type for a service object of type '<PlantopSegmentation-response>"
  "segment_plans_objects/PlantopSegmentationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlantopSegmentation-response)))
  "Returns string type for a service object of type 'PlantopSegmentation-response"
  "segment_plans_objects/PlantopSegmentationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlantopSegmentation-response>)))
  "Returns md5sum for a message object of type '<PlantopSegmentation-response>"
  "1200a1dffca987703a9988258c174cc5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlantopSegmentation-response)))
  "Returns md5sum for a message object of type 'PlantopSegmentation-response"
  "1200a1dffca987703a9988258c174cc5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlantopSegmentation-response>)))
  "Returns full string definition for message of type '<PlantopSegmentation-response>"
  (cl:format cl:nil "int32 NO_CLOUD_RECEIVED=1~%int32 NO_TABLE=2~%int32 OTHER_ERROR=3~%int32 SUCCESS=4~%object_recognition_msgs/Table table~%sensor_msgs/PointCloud2[] clusters~%sensor_msgs/Image[] cluster_images~%int32 result~%~%~%================================================================================~%MSG: object_recognition_msgs/Table~%# Informs that a planar table has been detected at a given location~%~%# The pose gives you the transform that take you to the coordinate system~%# of the table, with the origin somewhere in the table plane and the ~%# z axis normal to the plane~%geometry_msgs/PoseStamped pose~%~%# These values give you the observed extents of the table, along x and y,~%# in the table's own coordinate system (above)~%# there is no guarantee that the origin of the table coordinate system is~%# inside the boundary defined by these values. ~%float32 x_min~%float32 x_max~%float32 y_min~%float32 y_max~%~%# There is no guarantee that the table does NOT extend further than these ~%# values; this is just as far as we've observed it.~%~%# Newer table definition as triangle mesh of convex hull (relative to pose)~%shape_msgs/Mesh convex_hull~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: shape_msgs/Mesh~%# Definition of a mesh~%~%# list of triangles; the index values refer to positions in vertices[]~%MeshTriangle[] triangles~%~%# the actual vertices that make up the mesh~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: shape_msgs/MeshTriangle~%# Definition of a triangle's vertices~%uint32[3] vertex_indices~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlantopSegmentation-response)))
  "Returns full string definition for message of type 'PlantopSegmentation-response"
  (cl:format cl:nil "int32 NO_CLOUD_RECEIVED=1~%int32 NO_TABLE=2~%int32 OTHER_ERROR=3~%int32 SUCCESS=4~%object_recognition_msgs/Table table~%sensor_msgs/PointCloud2[] clusters~%sensor_msgs/Image[] cluster_images~%int32 result~%~%~%================================================================================~%MSG: object_recognition_msgs/Table~%# Informs that a planar table has been detected at a given location~%~%# The pose gives you the transform that take you to the coordinate system~%# of the table, with the origin somewhere in the table plane and the ~%# z axis normal to the plane~%geometry_msgs/PoseStamped pose~%~%# These values give you the observed extents of the table, along x and y,~%# in the table's own coordinate system (above)~%# there is no guarantee that the origin of the table coordinate system is~%# inside the boundary defined by these values. ~%float32 x_min~%float32 x_max~%float32 y_min~%float32 y_max~%~%# There is no guarantee that the table does NOT extend further than these ~%# values; this is just as far as we've observed it.~%~%# Newer table definition as triangle mesh of convex hull (relative to pose)~%shape_msgs/Mesh convex_hull~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: shape_msgs/Mesh~%# Definition of a mesh~%~%# list of triangles; the index values refer to positions in vertices[]~%MeshTriangle[] triangles~%~%# the actual vertices that make up the mesh~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: shape_msgs/MeshTriangle~%# Definition of a triangle's vertices~%uint32[3] vertex_indices~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlantopSegmentation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'table))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'clusters) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cluster_images) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlantopSegmentation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlantopSegmentation-response
    (cl:cons ':table (table msg))
    (cl:cons ':clusters (clusters msg))
    (cl:cons ':cluster_images (cluster_images msg))
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlantopSegmentation)))
  'PlantopSegmentation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlantopSegmentation)))
  'PlantopSegmentation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlantopSegmentation)))
  "Returns string type for a service object of type '<PlantopSegmentation>"
  "segment_plans_objects/PlantopSegmentation")