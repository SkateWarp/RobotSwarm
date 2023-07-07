;; Auto-generated. Do not edit!


(when (boundp 'hero_common::SetMotor)
  (if (not (find-package "HERO_COMMON"))
    (make-package "HERO_COMMON"))
  (shadow 'SetMotor (find-package "HERO_COMMON")))
(unless (find-package "HERO_COMMON::SETMOTOR")
  (make-package "HERO_COMMON::SETMOTOR"))
(unless (find-package "HERO_COMMON::SETMOTORREQUEST")
  (make-package "HERO_COMMON::SETMOTORREQUEST"))
(unless (find-package "HERO_COMMON::SETMOTORRESPONSE")
  (make-package "HERO_COMMON::SETMOTORRESPONSE"))

(in-package "ROS")





(defclass hero_common::SetMotorRequest
  :super ros::object
  :slots (_left_motor_pwm _right_motor_pwm ))

(defmethod hero_common::SetMotorRequest
  (:init
   (&key
    ((:left_motor_pwm __left_motor_pwm) 0)
    ((:right_motor_pwm __right_motor_pwm) 0)
    )
   (send-super :init)
   (setq _left_motor_pwm (round __left_motor_pwm))
   (setq _right_motor_pwm (round __right_motor_pwm))
   self)
  (:left_motor_pwm
   (&optional __left_motor_pwm)
   (if __left_motor_pwm (setq _left_motor_pwm __left_motor_pwm)) _left_motor_pwm)
  (:right_motor_pwm
   (&optional __right_motor_pwm)
   (if __right_motor_pwm (setq _right_motor_pwm __right_motor_pwm)) _right_motor_pwm)
  (:serialization-length
   ()
   (+
    ;; int16 _left_motor_pwm
    2
    ;; int16 _right_motor_pwm
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _left_motor_pwm
       (write-word _left_motor_pwm s)
     ;; int16 _right_motor_pwm
       (write-word _right_motor_pwm s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _left_motor_pwm
     (setq _left_motor_pwm (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _right_motor_pwm
     (setq _right_motor_pwm (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass hero_common::SetMotorResponse
  :super ros::object
  :slots (_success _message ))

(defmethod hero_common::SetMotorResponse
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

(defclass hero_common::SetMotor
  :super ros::object
  :slots ())

(setf (get hero_common::SetMotor :md5sum-) "07e32bced0fc14456815cdd097819b89")
(setf (get hero_common::SetMotor :datatype-) "hero_common/SetMotor")
(setf (get hero_common::SetMotor :request) hero_common::SetMotorRequest)
(setf (get hero_common::SetMotor :response) hero_common::SetMotorResponse)

(defmethod hero_common::SetMotorRequest
  (:response () (instance hero_common::SetMotorResponse :init)))

(setf (get hero_common::SetMotorRequest :md5sum-) "07e32bced0fc14456815cdd097819b89")
(setf (get hero_common::SetMotorRequest :datatype-) "hero_common/SetMotorRequest")
(setf (get hero_common::SetMotorRequest :definition-)
      "int16 left_motor_pwm # motor deadzone pwm
int16 right_motor_pwm # motor deadzone pwm
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")

(setf (get hero_common::SetMotorResponse :md5sum-) "07e32bced0fc14456815cdd097819b89")
(setf (get hero_common::SetMotorResponse :datatype-) "hero_common/SetMotorResponse")
(setf (get hero_common::SetMotorResponse :definition-)
      "int16 left_motor_pwm # motor deadzone pwm
int16 right_motor_pwm # motor deadzone pwm
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")



(provide :hero_common/SetMotor "07e32bced0fc14456815cdd097819b89")


