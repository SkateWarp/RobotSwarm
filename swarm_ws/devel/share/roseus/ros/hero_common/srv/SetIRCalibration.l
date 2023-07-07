;; Auto-generated. Do not edit!


(when (boundp 'hero_common::SetIRCalibration)
  (if (not (find-package "HERO_COMMON"))
    (make-package "HERO_COMMON"))
  (shadow 'SetIRCalibration (find-package "HERO_COMMON")))
(unless (find-package "HERO_COMMON::SETIRCALIBRATION")
  (make-package "HERO_COMMON::SETIRCALIBRATION"))
(unless (find-package "HERO_COMMON::SETIRCALIBRATIONREQUEST")
  (make-package "HERO_COMMON::SETIRCALIBRATIONREQUEST"))
(unless (find-package "HERO_COMMON::SETIRCALIBRATIONRESPONSE")
  (make-package "HERO_COMMON::SETIRCALIBRATIONRESPONSE"))

(in-package "ROS")





(defclass hero_common::SetIRCalibrationRequest
  :super ros::object
  :slots (_IR0 _IR1 _IR2 _IR3 _IR4 _IR5 _IR6 _IR7 ))

(defmethod hero_common::SetIRCalibrationRequest
  (:init
   (&key
    ((:IR0 __IR0) 0.0)
    ((:IR1 __IR1) 0.0)
    ((:IR2 __IR2) 0.0)
    ((:IR3 __IR3) 0.0)
    ((:IR4 __IR4) 0.0)
    ((:IR5 __IR5) 0.0)
    ((:IR6 __IR6) 0.0)
    ((:IR7 __IR7) 0.0)
    )
   (send-super :init)
   (setq _IR0 (float __IR0))
   (setq _IR1 (float __IR1))
   (setq _IR2 (float __IR2))
   (setq _IR3 (float __IR3))
   (setq _IR4 (float __IR4))
   (setq _IR5 (float __IR5))
   (setq _IR6 (float __IR6))
   (setq _IR7 (float __IR7))
   self)
  (:IR0
   (&optional __IR0)
   (if __IR0 (setq _IR0 __IR0)) _IR0)
  (:IR1
   (&optional __IR1)
   (if __IR1 (setq _IR1 __IR1)) _IR1)
  (:IR2
   (&optional __IR2)
   (if __IR2 (setq _IR2 __IR2)) _IR2)
  (:IR3
   (&optional __IR3)
   (if __IR3 (setq _IR3 __IR3)) _IR3)
  (:IR4
   (&optional __IR4)
   (if __IR4 (setq _IR4 __IR4)) _IR4)
  (:IR5
   (&optional __IR5)
   (if __IR5 (setq _IR5 __IR5)) _IR5)
  (:IR6
   (&optional __IR6)
   (if __IR6 (setq _IR6 __IR6)) _IR6)
  (:IR7
   (&optional __IR7)
   (if __IR7 (setq _IR7 __IR7)) _IR7)
  (:serialization-length
   ()
   (+
    ;; float64 _IR0
    8
    ;; float64 _IR1
    8
    ;; float64 _IR2
    8
    ;; float64 _IR3
    8
    ;; float64 _IR4
    8
    ;; float64 _IR5
    8
    ;; float64 _IR6
    8
    ;; float64 _IR7
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _IR0
       (sys::poke _IR0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _IR1
       (sys::poke _IR1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _IR2
       (sys::poke _IR2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _IR3
       (sys::poke _IR3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _IR4
       (sys::poke _IR4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _IR5
       (sys::poke _IR5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _IR6
       (sys::poke _IR6 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _IR7
       (sys::poke _IR7 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _IR0
     (setq _IR0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _IR1
     (setq _IR1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _IR2
     (setq _IR2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _IR3
     (setq _IR3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _IR4
     (setq _IR4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _IR5
     (setq _IR5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _IR6
     (setq _IR6 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _IR7
     (setq _IR7 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass hero_common::SetIRCalibrationResponse
  :super ros::object
  :slots (_success _message ))

(defmethod hero_common::SetIRCalibrationResponse
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

(defclass hero_common::SetIRCalibration
  :super ros::object
  :slots ())

(setf (get hero_common::SetIRCalibration :md5sum-) "fc55fd34ad2ef676815ba2e14521ac73")
(setf (get hero_common::SetIRCalibration :datatype-) "hero_common/SetIRCalibration")
(setf (get hero_common::SetIRCalibration :request) hero_common::SetIRCalibrationRequest)
(setf (get hero_common::SetIRCalibration :response) hero_common::SetIRCalibrationResponse)

(defmethod hero_common::SetIRCalibrationRequest
  (:response () (instance hero_common::SetIRCalibrationResponse :init)))

(setf (get hero_common::SetIRCalibrationRequest :md5sum-) "fc55fd34ad2ef676815ba2e14521ac73")
(setf (get hero_common::SetIRCalibrationRequest :datatype-) "hero_common/SetIRCalibrationRequest")
(setf (get hero_common::SetIRCalibrationRequest :definition-)
      "float64 IR0 # alpha value for IR0
float64 IR1 # alpha value for IR1
float64 IR2 # alpha value for IR2
float64 IR3 # alpha value for IR3
float64 IR4 # alpha value for IR4
float64 IR5 # alpha value for IR5
float64 IR6 # alpha value for IR6
float64 IR7 # alpha value for IR7
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")

(setf (get hero_common::SetIRCalibrationResponse :md5sum-) "fc55fd34ad2ef676815ba2e14521ac73")
(setf (get hero_common::SetIRCalibrationResponse :datatype-) "hero_common/SetIRCalibrationResponse")
(setf (get hero_common::SetIRCalibrationResponse :definition-)
      "float64 IR0 # alpha value for IR0
float64 IR1 # alpha value for IR1
float64 IR2 # alpha value for IR2
float64 IR3 # alpha value for IR3
float64 IR4 # alpha value for IR4
float64 IR5 # alpha value for IR5
float64 IR6 # alpha value for IR6
float64 IR7 # alpha value for IR7
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")



(provide :hero_common/SetIRCalibration "fc55fd34ad2ef676815ba2e14521ac73")


