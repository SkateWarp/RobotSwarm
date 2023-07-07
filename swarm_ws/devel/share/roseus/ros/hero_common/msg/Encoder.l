;; Auto-generated. Do not edit!


(when (boundp 'hero_common::Encoder)
  (if (not (find-package "HERO_COMMON"))
    (make-package "HERO_COMMON"))
  (shadow 'Encoder (find-package "HERO_COMMON")))
(unless (find-package "HERO_COMMON::ENCODER")
  (make-package "HERO_COMMON::ENCODER"))

(in-package "ROS")
;;//! \htmlinclude Encoder.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hero_common::Encoder
  :super ros::object
  :slots (_header _left_ticks _right_ticks _left_diff _right_diff _left_dist _right_dist _timestep _left_speed _right_speed _left_speed_filtered _right_speed_filtered ))

(defmethod hero_common::Encoder
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_ticks __left_ticks) 0.0)
    ((:right_ticks __right_ticks) 0.0)
    ((:left_diff __left_diff) 0.0)
    ((:right_diff __right_diff) 0.0)
    ((:left_dist __left_dist) 0.0)
    ((:right_dist __right_dist) 0.0)
    ((:timestep __timestep) 0.0)
    ((:left_speed __left_speed) 0.0)
    ((:right_speed __right_speed) 0.0)
    ((:left_speed_filtered __left_speed_filtered) 0.0)
    ((:right_speed_filtered __right_speed_filtered) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_ticks (float __left_ticks))
   (setq _right_ticks (float __right_ticks))
   (setq _left_diff (float __left_diff))
   (setq _right_diff (float __right_diff))
   (setq _left_dist (float __left_dist))
   (setq _right_dist (float __right_dist))
   (setq _timestep (float __timestep))
   (setq _left_speed (float __left_speed))
   (setq _right_speed (float __right_speed))
   (setq _left_speed_filtered (float __left_speed_filtered))
   (setq _right_speed_filtered (float __right_speed_filtered))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_ticks
   (&optional __left_ticks)
   (if __left_ticks (setq _left_ticks __left_ticks)) _left_ticks)
  (:right_ticks
   (&optional __right_ticks)
   (if __right_ticks (setq _right_ticks __right_ticks)) _right_ticks)
  (:left_diff
   (&optional __left_diff)
   (if __left_diff (setq _left_diff __left_diff)) _left_diff)
  (:right_diff
   (&optional __right_diff)
   (if __right_diff (setq _right_diff __right_diff)) _right_diff)
  (:left_dist
   (&optional __left_dist)
   (if __left_dist (setq _left_dist __left_dist)) _left_dist)
  (:right_dist
   (&optional __right_dist)
   (if __right_dist (setq _right_dist __right_dist)) _right_dist)
  (:timestep
   (&optional __timestep)
   (if __timestep (setq _timestep __timestep)) _timestep)
  (:left_speed
   (&optional __left_speed)
   (if __left_speed (setq _left_speed __left_speed)) _left_speed)
  (:right_speed
   (&optional __right_speed)
   (if __right_speed (setq _right_speed __right_speed)) _right_speed)
  (:left_speed_filtered
   (&optional __left_speed_filtered)
   (if __left_speed_filtered (setq _left_speed_filtered __left_speed_filtered)) _left_speed_filtered)
  (:right_speed_filtered
   (&optional __right_speed_filtered)
   (if __right_speed_filtered (setq _right_speed_filtered __right_speed_filtered)) _right_speed_filtered)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _left_ticks
    8
    ;; float64 _right_ticks
    8
    ;; float64 _left_diff
    8
    ;; float64 _right_diff
    8
    ;; float64 _left_dist
    8
    ;; float64 _right_dist
    8
    ;; float64 _timestep
    8
    ;; float64 _left_speed
    8
    ;; float64 _right_speed
    8
    ;; float64 _left_speed_filtered
    8
    ;; float64 _right_speed_filtered
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _left_ticks
       (sys::poke _left_ticks (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_ticks
       (sys::poke _right_ticks (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_diff
       (sys::poke _left_diff (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_diff
       (sys::poke _right_diff (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_dist
       (sys::poke _left_dist (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_dist
       (sys::poke _right_dist (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _timestep
       (sys::poke _timestep (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_speed
       (sys::poke _left_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_speed
       (sys::poke _right_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_speed_filtered
       (sys::poke _left_speed_filtered (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_speed_filtered
       (sys::poke _right_speed_filtered (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _left_ticks
     (setq _left_ticks (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_ticks
     (setq _right_ticks (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_diff
     (setq _left_diff (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_diff
     (setq _right_diff (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_dist
     (setq _left_dist (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_dist
     (setq _right_dist (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _timestep
     (setq _timestep (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_speed
     (setq _left_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_speed
     (setq _right_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_speed_filtered
     (setq _left_speed_filtered (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_speed_filtered
     (setq _right_speed_filtered (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get hero_common::Encoder :md5sum-) "b5586cdbae9b740afc6a1f403a2d7dde")
(setf (get hero_common::Encoder :datatype-) "hero_common/Encoder")
(setf (get hero_common::Encoder :definition-)
      "Header header

float64 left_ticks
float64 right_ticks
float64 left_diff
float64 right_diff
float64 left_dist
float64 right_dist
float64 timestep
float64 left_speed
float64 right_speed
float64 left_speed_filtered
float64 right_speed_filtered
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

")



(provide :hero_common/Encoder "b5586cdbae9b740afc6a1f403a2d7dde")


