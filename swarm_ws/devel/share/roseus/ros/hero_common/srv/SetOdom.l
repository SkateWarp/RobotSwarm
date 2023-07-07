;; Auto-generated. Do not edit!


(when (boundp 'hero_common::SetOdom)
  (if (not (find-package "HERO_COMMON"))
    (make-package "HERO_COMMON"))
  (shadow 'SetOdom (find-package "HERO_COMMON")))
(unless (find-package "HERO_COMMON::SETODOM")
  (make-package "HERO_COMMON::SETODOM"))
(unless (find-package "HERO_COMMON::SETODOMREQUEST")
  (make-package "HERO_COMMON::SETODOMREQUEST"))
(unless (find-package "HERO_COMMON::SETODOMRESPONSE")
  (make-package "HERO_COMMON::SETODOMRESPONSE"))

(in-package "ROS")





(defclass hero_common::SetOdomRequest
  :super ros::object
  :slots (_x _y _theta ))

(defmethod hero_common::SetOdomRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass hero_common::SetOdomResponse
  :super ros::object
  :slots (_success _message ))

(defmethod hero_common::SetOdomResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass hero_common::SetOdom
  :super ros::object
  :slots ())

(setf (get hero_common::SetOdom :md5sum-) "4ed90569b1afd039b963e42a12637d2b")
(setf (get hero_common::SetOdom :datatype-) "hero_common/SetOdom")
(setf (get hero_common::SetOdom :request) hero_common::SetOdomRequest)
(setf (get hero_common::SetOdom :response) hero_common::SetOdomResponse)

(defmethod hero_common::SetOdomRequest
  (:response () (instance hero_common::SetOdomResponse :init)))

(setf (get hero_common::SetOdomRequest :md5sum-) "4ed90569b1afd039b963e42a12637d2b")
(setf (get hero_common::SetOdomRequest :datatype-) "hero_common/SetOdomRequest")
(setf (get hero_common::SetOdomRequest :definition-)
      "float64 x # proportional term
float64 y # intergrative term
float64 theta # derivative term
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")

(setf (get hero_common::SetOdomResponse :md5sum-) "4ed90569b1afd039b963e42a12637d2b")
(setf (get hero_common::SetOdomResponse :datatype-) "hero_common/SetOdomResponse")
(setf (get hero_common::SetOdomResponse :definition-)
      "float64 x # proportional term
float64 y # intergrative term
float64 theta # derivative term
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")



(provide :hero_common/SetOdom "4ed90569b1afd039b963e42a12637d2b")


