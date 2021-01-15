; Auto-generated. Do not edit!


(cl:in-package carla_msgs-msg)


;//! \htmlinclude CarlaRadarMeasurement.msg.html

(cl:defclass <CarlaRadarMeasurement> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector carla_msgs-msg:CarlaRadarDetection)
   :initform (cl:make-array 0 :element-type 'carla_msgs-msg:CarlaRadarDetection :initial-element (cl:make-instance 'carla_msgs-msg:CarlaRadarDetection))))
)

(cl:defclass CarlaRadarMeasurement (<CarlaRadarMeasurement>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CarlaRadarMeasurement>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CarlaRadarMeasurement)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name carla_msgs-msg:<CarlaRadarMeasurement> is deprecated: use carla_msgs-msg:CarlaRadarMeasurement instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CarlaRadarMeasurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carla_msgs-msg:header-val is deprecated.  Use carla_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <CarlaRadarMeasurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carla_msgs-msg:detections-val is deprecated.  Use carla_msgs-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CarlaRadarMeasurement>) ostream)
  "Serializes a message object of type '<CarlaRadarMeasurement>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CarlaRadarMeasurement>) istream)
  "Deserializes a message object of type '<CarlaRadarMeasurement>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'carla_msgs-msg:CarlaRadarDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CarlaRadarMeasurement>)))
  "Returns string type for a message object of type '<CarlaRadarMeasurement>"
  "carla_msgs/CarlaRadarMeasurement")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CarlaRadarMeasurement)))
  "Returns string type for a message object of type 'CarlaRadarMeasurement"
  "carla_msgs/CarlaRadarMeasurement")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CarlaRadarMeasurement>)))
  "Returns md5sum for a message object of type '<CarlaRadarMeasurement>"
  "1ea43324c6693f5d104fc272e6fdd08e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CarlaRadarMeasurement)))
  "Returns md5sum for a message object of type 'CarlaRadarMeasurement"
  "1ea43324c6693f5d104fc272e6fdd08e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CarlaRadarMeasurement>)))
  "Returns full string definition for message of type '<CarlaRadarMeasurement>"
  (cl:format cl:nil "#~%# Copyright (c) 2020 Intel Corporation.~%#~%# This work is licensed under the terms of the MIT license.~%# For a copy, see <https://opensource.org/licenses/MIT>.~%#~%~%Header header~%~%CarlaRadarDetection[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: carla_msgs/CarlaRadarDetection~%#~%# Copyright (c) 2020 Intel Corporation.~%#~%# This work is licensed under the terms of the MIT license.~%# For a copy, see <https://opensource.org/licenses/MIT>.~%#~%~%float32 altitude~%float32 azimuth~%float32 depth~%float32 velocity~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CarlaRadarMeasurement)))
  "Returns full string definition for message of type 'CarlaRadarMeasurement"
  (cl:format cl:nil "#~%# Copyright (c) 2020 Intel Corporation.~%#~%# This work is licensed under the terms of the MIT license.~%# For a copy, see <https://opensource.org/licenses/MIT>.~%#~%~%Header header~%~%CarlaRadarDetection[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: carla_msgs/CarlaRadarDetection~%#~%# Copyright (c) 2020 Intel Corporation.~%#~%# This work is licensed under the terms of the MIT license.~%# For a copy, see <https://opensource.org/licenses/MIT>.~%#~%~%float32 altitude~%float32 azimuth~%float32 depth~%float32 velocity~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CarlaRadarMeasurement>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CarlaRadarMeasurement>))
  "Converts a ROS message object to a list"
  (cl:list 'CarlaRadarMeasurement
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
))
