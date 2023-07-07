;; Auto-generated. Do not edit!


(when (boundp 'hero_common::SetFrequency)
  (if (not (find-package "HERO_COMMON"))
    (make-package "HERO_COMMON"))
  (shadow 'SetFrequency (find-package "HERO_COMMON")))
(unless (find-package "HERO_COMMON::SETFREQUENCY")
  (make-package "HERO_COMMON::SETFREQUENCY"))
(unless (find-package "HERO_COMMON::SETFREQUENCYREQUEST")
  (make-package "HERO_COMMON::SETFREQUENCYREQUEST"))
(unless (find-package "HERO_COMMON::SETFREQUENCYRESPONSE")
  (make-package "HERO_COMMON::SETFREQUENCYRESPONSE"))

(in-package "ROS")





(defclass hero_common::SetFrequencyRequest
  :super ros::object
  :slots (_frequency ))

(defmethod hero_common::SetFrequencyRequest
  (:init
   (&key
    ((:frequency __frequency) 0.0)
    )
   (send-super :init)
   (setq _frequency (float __frequency))
   self)
  (:frequency
   (&optional __frequency)
   (if __frequency (setq _frequency __frequency)) _frequency)
  (:serialization-length
   ()
   (+
    ;; float64 _frequency
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _frequency
       (sys::poke _frequency (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _frequency
     (setq _frequency (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass hero_common::SetFrequencyResponse
  :super ros::object
  :slots (_success _message ))

(defmethod hero_common::SetFrequencyResponse
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

(defclass hero_common::SetFrequency
  :super ros::object
  :slots ())

(setf (get hero_common::SetFrequency :md5sum-) "ff3ae8a6187feadac85089456faed43f")
(setf (get hero_common::SetFrequency :datatype-) "hero_common/SetFrequency")
(setf (get hero_common::SetFrequency :request) hero_common::SetFrequencyRequest)
(setf (get hero_common::SetFrequency :response) hero_common::SetFrequencyResponse)

(defmethod hero_common::SetFrequencyRequest
  (:response () (instance hero_common::SetFrequencyResponse :init)))

(setf (get hero_common::SetFrequencyRequest :md5sum-) "ff3ae8a6187feadac85089456faed43f")
(setf (get hero_common::SetFrequencyRequest :datatype-) "hero_common/SetFrequencyRequest")
(setf (get hero_common::SetFrequencyRequest :definition-)
      "float64 frequency # loop-frequency in Hz
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")

(setf (get hero_common::SetFrequencyResponse :md5sum-) "ff3ae8a6187feadac85089456faed43f")
(setf (get hero_common::SetFrequencyResponse :datatype-) "hero_common/SetFrequencyResponse")
(setf (get hero_common::SetFrequencyResponse :definition-)
      "float64 frequency # loop-frequency in Hz
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")



(provide :hero_common/SetFrequency "ff3ae8a6187feadac85089456faed43f")


