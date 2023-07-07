;; Auto-generated. Do not edit!


(when (boundp 'hero_common::SetPID)
  (if (not (find-package "HERO_COMMON"))
    (make-package "HERO_COMMON"))
  (shadow 'SetPID (find-package "HERO_COMMON")))
(unless (find-package "HERO_COMMON::SETPID")
  (make-package "HERO_COMMON::SETPID"))
(unless (find-package "HERO_COMMON::SETPIDREQUEST")
  (make-package "HERO_COMMON::SETPIDREQUEST"))
(unless (find-package "HERO_COMMON::SETPIDRESPONSE")
  (make-package "HERO_COMMON::SETPIDRESPONSE"))

(in-package "ROS")





(defclass hero_common::SetPIDRequest
  :super ros::object
  :slots (_lkp _lki _lkd _rkp _rki _rkd ))

(defmethod hero_common::SetPIDRequest
  (:init
   (&key
    ((:lkp __lkp) 0.0)
    ((:lki __lki) 0.0)
    ((:lkd __lkd) 0.0)
    ((:rkp __rkp) 0.0)
    ((:rki __rki) 0.0)
    ((:rkd __rkd) 0.0)
    )
   (send-super :init)
   (setq _lkp (float __lkp))
   (setq _lki (float __lki))
   (setq _lkd (float __lkd))
   (setq _rkp (float __rkp))
   (setq _rki (float __rki))
   (setq _rkd (float __rkd))
   self)
  (:lkp
   (&optional __lkp)
   (if __lkp (setq _lkp __lkp)) _lkp)
  (:lki
   (&optional __lki)
   (if __lki (setq _lki __lki)) _lki)
  (:lkd
   (&optional __lkd)
   (if __lkd (setq _lkd __lkd)) _lkd)
  (:rkp
   (&optional __rkp)
   (if __rkp (setq _rkp __rkp)) _rkp)
  (:rki
   (&optional __rki)
   (if __rki (setq _rki __rki)) _rki)
  (:rkd
   (&optional __rkd)
   (if __rkd (setq _rkd __rkd)) _rkd)
  (:serialization-length
   ()
   (+
    ;; float32 _lkp
    4
    ;; float32 _lki
    4
    ;; float32 _lkd
    4
    ;; float32 _rkp
    4
    ;; float32 _rki
    4
    ;; float32 _rkd
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _lkp
       (sys::poke _lkp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lki
       (sys::poke _lki (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lkd
       (sys::poke _lkd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rkp
       (sys::poke _rkp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rki
       (sys::poke _rki (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rkd
       (sys::poke _rkd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _lkp
     (setq _lkp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lki
     (setq _lki (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lkd
     (setq _lkd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rkp
     (setq _rkp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rki
     (setq _rki (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rkd
     (setq _rkd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass hero_common::SetPIDResponse
  :super ros::object
  :slots (_success _message ))

(defmethod hero_common::SetPIDResponse
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

(defclass hero_common::SetPID
  :super ros::object
  :slots ())

(setf (get hero_common::SetPID :md5sum-) "69c16eccc66fd6b477afb5e5500b4b47")
(setf (get hero_common::SetPID :datatype-) "hero_common/SetPID")
(setf (get hero_common::SetPID :request) hero_common::SetPIDRequest)
(setf (get hero_common::SetPID :response) hero_common::SetPIDResponse)

(defmethod hero_common::SetPIDRequest
  (:response () (instance hero_common::SetPIDResponse :init)))

(setf (get hero_common::SetPIDRequest :md5sum-) "69c16eccc66fd6b477afb5e5500b4b47")
(setf (get hero_common::SetPIDRequest :datatype-) "hero_common/SetPIDRequest")
(setf (get hero_common::SetPIDRequest :definition-)
      "float32 lkp # proportional term left motor
float32 lki # intergrative term left motor
float32 lkd # derivative term left motor
float32 rkp # proportional term right motor
float32 rki # intergrative term right motor
float32 rkd # derivative term right motor
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")

(setf (get hero_common::SetPIDResponse :md5sum-) "69c16eccc66fd6b477afb5e5500b4b47")
(setf (get hero_common::SetPIDResponse :datatype-) "hero_common/SetPIDResponse")
(setf (get hero_common::SetPIDResponse :definition-)
      "float32 lkp # proportional term left motor
float32 lki # intergrative term left motor
float32 lkd # derivative term left motor
float32 rkp # proportional term right motor
float32 rki # intergrative term right motor
float32 rkd # derivative term right motor
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")



(provide :hero_common/SetPID "69c16eccc66fd6b477afb5e5500b4b47")


