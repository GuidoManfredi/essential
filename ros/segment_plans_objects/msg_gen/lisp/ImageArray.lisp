; Auto-generated. Do not edit!


(cl:in-package segment_plans_objects-msg)


;//! \htmlinclude ImageArray.msg.html

(cl:defclass <ImageArray> (roslisp-msg-protocol:ros-message)
  ((array
    :reader array
    :initarg :array
    :type (cl:vector sensor_msgs-msg:Image)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:Image :initial-element (cl:make-instance 'sensor_msgs-msg:Image))))
)

(cl:defclass ImageArray (<ImageArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name segment_plans_objects-msg:<ImageArray> is deprecated: use segment_plans_objects-msg:ImageArray instead.")))

(cl:ensure-generic-function 'array-val :lambda-list '(m))
(cl:defmethod array-val ((m <ImageArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segment_plans_objects-msg:array-val is deprecated.  Use segment_plans_objects-msg:array instead.")
  (array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageArray>) ostream)
  "Serializes a message object of type '<ImageArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageArray>) istream)
  "Deserializes a message object of type '<ImageArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:Image))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageArray>)))
  "Returns string type for a message object of type '<ImageArray>"
  "segment_plans_objects/ImageArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageArray)))
  "Returns string type for a message object of type 'ImageArray"
  "segment_plans_objects/ImageArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageArray>)))
  "Returns md5sum for a message object of type '<ImageArray>"
  "2d45007bb1ef655f17889f52265cf557")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageArray)))
  "Returns md5sum for a message object of type 'ImageArray"
  "2d45007bb1ef655f17889f52265cf557")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageArray>)))
  "Returns full string definition for message of type '<ImageArray>"
  (cl:format cl:nil "sensor_msgs/Image[] array~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageArray)))
  "Returns full string definition for message of type 'ImageArray"
  (cl:format cl:nil "sensor_msgs/Image[] array~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageArray
    (cl:cons ':array (array msg))
))
