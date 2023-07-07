;; Auto-generated. Do not edit!


(when (boundp 'hero_common::Motor)
  (if (not (find-package "HERO_COMMON"))
    (make-package "HERO_COMMON"))
  (shadow 'Motor (find-package "HERO_COMMON")))
(unless (find-package "HERO_COMMON::MOTOR")
  (make-package "HERO_COMMON::MOTOR"))

(in-package "ROS")
;;//! \htmlinclude Motor.msg.html


(defclass hero_common::Motor
  :super ros::object
  :slots (_left_motor_pwm _right_motor_pwm ))

(defmethod hero_common::Motor
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

(setf (get hero_common::Motor :md5sum-) "44d4249d207ce6a89de3d72080e4d0b5")
(setf (get hero_common::Motor :datatype-) "hero_common/Motor")
(setf (get hero_common::Motor :definition-)
      "# Left and right motors PWM - range from 700 to 2300
int16 left_motor_pwm
int16 right_motor_pwm

")



(provide :hero_common/Motor "44d4249d207ce6a89de3d72080e4d0b5")


