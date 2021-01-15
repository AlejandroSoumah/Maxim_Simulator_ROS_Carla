;; Auto-generated. Do not edit!


(when (boundp 'carla_msgs::CarlaRadarMeasurement)
  (if (not (find-package "CARLA_MSGS"))
    (make-package "CARLA_MSGS"))
  (shadow 'CarlaRadarMeasurement (find-package "CARLA_MSGS")))
(unless (find-package "CARLA_MSGS::CARLARADARMEASUREMENT")
  (make-package "CARLA_MSGS::CARLARADARMEASUREMENT"))

(in-package "ROS")
;;//! \htmlinclude CarlaRadarMeasurement.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass carla_msgs::CarlaRadarMeasurement
  :super ros::object
  :slots (_header _detections ))

(defmethod carla_msgs::CarlaRadarMeasurement
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detections __detections) (let (r) (dotimes (i 0) (push (instance carla_msgs::CarlaRadarDetection :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; carla_msgs/CarlaRadarDetection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; carla_msgs/CarlaRadarDetection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; carla_msgs/CarlaRadarDetection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance carla_msgs::CarlaRadarDetection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get carla_msgs::CarlaRadarMeasurement :md5sum-) "1ea43324c6693f5d104fc272e6fdd08e")
(setf (get carla_msgs::CarlaRadarMeasurement :datatype-) "carla_msgs/CarlaRadarMeasurement")
(setf (get carla_msgs::CarlaRadarMeasurement :definition-)
      "#
# Copyright (c) 2020 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

Header header

CarlaRadarDetection[] detections
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: carla_msgs/CarlaRadarDetection
#
# Copyright (c) 2020 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

float32 altitude
float32 azimuth
float32 depth
float32 velocity
")



(provide :carla_msgs/CarlaRadarMeasurement "1ea43324c6693f5d104fc272e6fdd08e")


