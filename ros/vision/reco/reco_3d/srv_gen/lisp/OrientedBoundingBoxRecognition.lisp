; Auto-generated. Do not edit!


(cl:in-package reco_3d-srv)


;//! \htmlinclude OrientedBoundingBoxRecognition-request.msg.html

(cl:defclass <OrientedBoundingBoxRecognition-request> (roslisp-msg-protocol:ros-message)
  ((cluster
    :reader cluster
    :initarg :cluster
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass OrientedBoundingBoxRecognition-request (<OrientedBoundingBoxRecognition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OrientedBoundingBoxRecognition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OrientedBoundingBoxRecognition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reco_3d-srv:<OrientedBoundingBoxRecognition-request> is deprecated: use reco_3d-srv:OrientedBoundingBoxRecognition-request instead.")))

(cl:ensure-generic-function 'cluster-val :lambda-list '(m))
(cl:defmethod cluster-val ((m <OrientedBoundingBoxRecognition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reco_3d-srv:cluster-val is deprecated.  Use reco_3d-srv:cluster instead.")
  (cluster m))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <OrientedBoundingBoxRecognition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reco_3d-srv:names-val is deprecated.  Use reco_3d-srv:names instead.")
  (names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OrientedBoundingBoxRecognition-request>) ostream)
  "Serializes a message object of type '<OrientedBoundingBoxRecognition-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cluster) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'names))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OrientedBoundingBoxRecognition-request>) istream)
  "Deserializes a message object of type '<OrientedBoundingBoxRecognition-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cluster) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OrientedBoundingBoxRecognition-request>)))
  "Returns string type for a service object of type '<OrientedBoundingBoxRecognition-request>"
  "reco_3d/OrientedBoundingBoxRecognitionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OrientedBoundingBoxRecognition-request)))
  "Returns string type for a service object of type 'OrientedBoundingBoxRecognition-request"
  "reco_3d/OrientedBoundingBoxRecognitionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OrientedBoundingBoxRecognition-request>)))
  "Returns md5sum for a message object of type '<OrientedBoundingBoxRecognition-request>"
  "548454f15a44418037250e6b978bad35")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OrientedBoundingBoxRecognition-request)))
  "Returns md5sum for a message object of type 'OrientedBoundingBoxRecognition-request"
  "548454f15a44418037250e6b978bad35")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OrientedBoundingBoxRecognition-request>)))
  "Returns full string definition for message of type '<OrientedBoundingBoxRecognition-request>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cluster~%string[] names~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OrientedBoundingBoxRecognition-request)))
  "Returns full string definition for message of type 'OrientedBoundingBoxRecognition-request"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cluster~%string[] names~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OrientedBoundingBoxRecognition-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cluster))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OrientedBoundingBoxRecognition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OrientedBoundingBoxRecognition-request
    (cl:cons ':cluster (cluster msg))
    (cl:cons ':names (names msg))
))
;//! \htmlinclude OrientedBoundingBoxRecognition-response.msg.html

(cl:defclass <OrientedBoundingBoxRecognition-response> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (box_dims
    :reader box_dims
    :initarg :box_dims
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass OrientedBoundingBoxRecognition-response (<OrientedBoundingBoxRecognition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OrientedBoundingBoxRecognition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OrientedBoundingBoxRecognition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reco_3d-srv:<OrientedBoundingBoxRecognition-response> is deprecated: use reco_3d-srv:OrientedBoundingBoxRecognition-response instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <OrientedBoundingBoxRecognition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reco_3d-srv:pose-val is deprecated.  Use reco_3d-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'box_dims-val :lambda-list '(m))
(cl:defmethod box_dims-val ((m <OrientedBoundingBoxRecognition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reco_3d-srv:box_dims-val is deprecated.  Use reco_3d-srv:box_dims instead.")
  (box_dims m))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <OrientedBoundingBoxRecognition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reco_3d-srv:names-val is deprecated.  Use reco_3d-srv:names instead.")
  (names m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <OrientedBoundingBoxRecognition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reco_3d-srv:result-val is deprecated.  Use reco_3d-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<OrientedBoundingBoxRecognition-response>)))
    "Constants for message type '<OrientedBoundingBoxRecognition-response>"
  '((:SUCCESS . 0)
    (:ERROR . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'OrientedBoundingBoxRecognition-response)))
    "Constants for message type 'OrientedBoundingBoxRecognition-response"
  '((:SUCCESS . 0)
    (:ERROR . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OrientedBoundingBoxRecognition-response>) ostream)
  "Serializes a message object of type '<OrientedBoundingBoxRecognition-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'box_dims) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'names))
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OrientedBoundingBoxRecognition-response>) istream)
  "Deserializes a message object of type '<OrientedBoundingBoxRecognition-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'box_dims) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OrientedBoundingBoxRecognition-response>)))
  "Returns string type for a service object of type '<OrientedBoundingBoxRecognition-response>"
  "reco_3d/OrientedBoundingBoxRecognitionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OrientedBoundingBoxRecognition-response)))
  "Returns string type for a service object of type 'OrientedBoundingBoxRecognition-response"
  "reco_3d/OrientedBoundingBoxRecognitionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OrientedBoundingBoxRecognition-response>)))
  "Returns md5sum for a message object of type '<OrientedBoundingBoxRecognition-response>"
  "548454f15a44418037250e6b978bad35")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OrientedBoundingBoxRecognition-response)))
  "Returns md5sum for a message object of type 'OrientedBoundingBoxRecognition-response"
  "548454f15a44418037250e6b978bad35")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OrientedBoundingBoxRecognition-response>)))
  "Returns full string definition for message of type '<OrientedBoundingBoxRecognition-response>"
  (cl:format cl:nil "~%int32 SUCCESS=0~%int32 ERROR=1~%~%~%geometry_msgs/PoseStamped pose~%~%geometry_msgs/Vector3 box_dims~%~%string[] names~%~%~%int32 result~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OrientedBoundingBoxRecognition-response)))
  "Returns full string definition for message of type 'OrientedBoundingBoxRecognition-response"
  (cl:format cl:nil "~%int32 SUCCESS=0~%int32 ERROR=1~%~%~%geometry_msgs/PoseStamped pose~%~%geometry_msgs/Vector3 box_dims~%~%string[] names~%~%~%int32 result~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OrientedBoundingBoxRecognition-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'box_dims))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OrientedBoundingBoxRecognition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OrientedBoundingBoxRecognition-response
    (cl:cons ':pose (pose msg))
    (cl:cons ':box_dims (box_dims msg))
    (cl:cons ':names (names msg))
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OrientedBoundingBoxRecognition)))
  'OrientedBoundingBoxRecognition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OrientedBoundingBoxRecognition)))
  'OrientedBoundingBoxRecognition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OrientedBoundingBoxRecognition)))
  "Returns string type for a service object of type '<OrientedBoundingBoxRecognition>"
  "reco_3d/OrientedBoundingBoxRecognition")