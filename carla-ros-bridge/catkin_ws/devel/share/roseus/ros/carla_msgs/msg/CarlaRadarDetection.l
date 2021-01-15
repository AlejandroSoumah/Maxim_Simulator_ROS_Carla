;; Auto-generated. Do not edit!


(when (boundp 'carla_msgs::CarlaRadarDetection)
  (if (not (find-package "CARLA_MSGS"))
    (make-package "CARLA_MSGS"))
  (shadow 'CarlaRadarDetection (find-package "CARLA_MSGS")))
(unless (find-package "CARLA_MSGS::CARLARADARDETECTION")
  (make-package "CARLA_MSGS::CARLARADARDETECTION"))

(in-package "ROS")
;;//! \htmlinclude CarlaRadarDetection.msg.html


(defclass carla_msgs::CarlaRadarDetection
  :super ros::object
  :slots (_altitude _azimuth _depth _velocity ))

(defmethod carla_msgs::CarlaRadarDetection
  (:init
   (&key
    ((:altitude __altitude) 0.0)
    ((:azimuth __azimuth) 0.0)
    ((:depth __depth) 0.0)
    ((:velocity __velocity) 0.0)
    )
   (send-super :init)
   (setq _altitude (float __altitude))
   (setq _azimuth (float __azimuth))
   (setq _depth (float __depth))
   (setq _velocity (float __velocity))
   self)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:azimuth
   (&optional __azimuth)
   (if __azimuth (setq _azimuth __azimuth)) _azimuth)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:serialization-length
   ()
   (+
    ;; float32 _altitude
    4
    ;; float32 _azimuth
    4
    ;; float32 _depth
    4
    ;; float32 _velocity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _azimuth
       (sys::poke _azimuth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _altitude
     (setq _altitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _azimuth
     (setq _azimuth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _depth
     (setq _depth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get carla_msgs::CarlaRadarDetection :md5sum-) "a3ece6192cae1ab73b9842e20a38284a")
(setf (get carla_msgs::CarlaRadarDetection :datatype-) "carla_msgs/CarlaRadarDetection")
(setf (get carla_msgs::CarlaRadarDetection :definition-)
      "#
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



(provide :carla_msgs/CarlaRadarDetection "a3ece6192cae1ab73b9842e20a38284a")


