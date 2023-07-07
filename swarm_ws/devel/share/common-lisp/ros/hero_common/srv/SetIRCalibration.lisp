; Auto-generated. Do not edit!


(cl:in-package hero_common-srv)


;//! \htmlinclude SetIRCalibration-request.msg.html

(cl:defclass <SetIRCalibration-request> (roslisp-msg-protocol:ros-message)
  ((IR0
    :reader IR0
    :initarg :IR0
    :type cl:float
    :initform 0.0)
   (IR1
    :reader IR1
    :initarg :IR1
    :type cl:float
    :initform 0.0)
   (IR2
    :reader IR2
    :initarg :IR2
    :type cl:float
    :initform 0.0)
   (IR3
    :reader IR3
    :initarg :IR3
    :type cl:float
    :initform 0.0)
   (IR4
    :reader IR4
    :initarg :IR4
    :type cl:float
    :initform 0.0)
   (IR5
    :reader IR5
    :initarg :IR5
    :type cl:float
    :initform 0.0)
   (IR6
    :reader IR6
    :initarg :IR6
    :type cl:float
    :initform 0.0)
   (IR7
    :reader IR7
    :initarg :IR7
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetIRCalibration-request (<SetIRCalibration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetIRCalibration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetIRCalibration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hero_common-srv:<SetIRCalibration-request> is deprecated: use hero_common-srv:SetIRCalibration-request instead.")))

(cl:ensure-generic-function 'IR0-val :lambda-list '(m))
(cl:defmethod IR0-val ((m <SetIRCalibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:IR0-val is deprecated.  Use hero_common-srv:IR0 instead.")
  (IR0 m))

(cl:ensure-generic-function 'IR1-val :lambda-list '(m))
(cl:defmethod IR1-val ((m <SetIRCalibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:IR1-val is deprecated.  Use hero_common-srv:IR1 instead.")
  (IR1 m))

(cl:ensure-generic-function 'IR2-val :lambda-list '(m))
(cl:defmethod IR2-val ((m <SetIRCalibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:IR2-val is deprecated.  Use hero_common-srv:IR2 instead.")
  (IR2 m))

(cl:ensure-generic-function 'IR3-val :lambda-list '(m))
(cl:defmethod IR3-val ((m <SetIRCalibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:IR3-val is deprecated.  Use hero_common-srv:IR3 instead.")
  (IR3 m))

(cl:ensure-generic-function 'IR4-val :lambda-list '(m))
(cl:defmethod IR4-val ((m <SetIRCalibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:IR4-val is deprecated.  Use hero_common-srv:IR4 instead.")
  (IR4 m))

(cl:ensure-generic-function 'IR5-val :lambda-list '(m))
(cl:defmethod IR5-val ((m <SetIRCalibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:IR5-val is deprecated.  Use hero_common-srv:IR5 instead.")
  (IR5 m))

(cl:ensure-generic-function 'IR6-val :lambda-list '(m))
(cl:defmethod IR6-val ((m <SetIRCalibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:IR6-val is deprecated.  Use hero_common-srv:IR6 instead.")
  (IR6 m))

(cl:ensure-generic-function 'IR7-val :lambda-list '(m))
(cl:defmethod IR7-val ((m <SetIRCalibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:IR7-val is deprecated.  Use hero_common-srv:IR7 instead.")
  (IR7 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetIRCalibration-request>) ostream)
  "Serializes a message object of type '<SetIRCalibration-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR6))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR7))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetIRCalibration-request>) istream)
  "Deserializes a message object of type '<SetIRCalibration-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR0) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR4) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR5) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR6) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR7) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetIRCalibration-request>)))
  "Returns string type for a service object of type '<SetIRCalibration-request>"
  "hero_common/SetIRCalibrationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIRCalibration-request)))
  "Returns string type for a service object of type 'SetIRCalibration-request"
  "hero_common/SetIRCalibrationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetIRCalibration-request>)))
  "Returns md5sum for a message object of type '<SetIRCalibration-request>"
  "fc55fd34ad2ef676815ba2e14521ac73")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetIRCalibration-request)))
  "Returns md5sum for a message object of type 'SetIRCalibration-request"
  "fc55fd34ad2ef676815ba2e14521ac73")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetIRCalibration-request>)))
  "Returns full string definition for message of type '<SetIRCalibration-request>"
  (cl:format cl:nil "float64 IR0 # alpha value for IR0~%float64 IR1 # alpha value for IR1~%float64 IR2 # alpha value for IR2~%float64 IR3 # alpha value for IR3~%float64 IR4 # alpha value for IR4~%float64 IR5 # alpha value for IR5~%float64 IR6 # alpha value for IR6~%float64 IR7 # alpha value for IR7~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetIRCalibration-request)))
  "Returns full string definition for message of type 'SetIRCalibration-request"
  (cl:format cl:nil "float64 IR0 # alpha value for IR0~%float64 IR1 # alpha value for IR1~%float64 IR2 # alpha value for IR2~%float64 IR3 # alpha value for IR3~%float64 IR4 # alpha value for IR4~%float64 IR5 # alpha value for IR5~%float64 IR6 # alpha value for IR6~%float64 IR7 # alpha value for IR7~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetIRCalibration-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetIRCalibration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetIRCalibration-request
    (cl:cons ':IR0 (IR0 msg))
    (cl:cons ':IR1 (IR1 msg))
    (cl:cons ':IR2 (IR2 msg))
    (cl:cons ':IR3 (IR3 msg))
    (cl:cons ':IR4 (IR4 msg))
    (cl:cons ':IR5 (IR5 msg))
    (cl:cons ':IR6 (IR6 msg))
    (cl:cons ':IR7 (IR7 msg))
))
;//! \htmlinclude SetIRCalibration-response.msg.html

(cl:defclass <SetIRCalibration-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SetIRCalibration-response (<SetIRCalibration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetIRCalibration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetIRCalibration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hero_common-srv:<SetIRCalibration-response> is deprecated: use hero_common-srv:SetIRCalibration-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetIRCalibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:success-val is deprecated.  Use hero_common-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetIRCalibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:message-val is deprecated.  Use hero_common-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetIRCalibration-response>) ostream)
  "Serializes a message object of type '<SetIRCalibration-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetIRCalibration-response>) istream)
  "Deserializes a message object of type '<SetIRCalibration-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetIRCalibration-response>)))
  "Returns string type for a service object of type '<SetIRCalibration-response>"
  "hero_common/SetIRCalibrationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIRCalibration-response)))
  "Returns string type for a service object of type 'SetIRCalibration-response"
  "hero_common/SetIRCalibrationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetIRCalibration-response>)))
  "Returns md5sum for a message object of type '<SetIRCalibration-response>"
  "fc55fd34ad2ef676815ba2e14521ac73")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetIRCalibration-response)))
  "Returns md5sum for a message object of type 'SetIRCalibration-response"
  "fc55fd34ad2ef676815ba2e14521ac73")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetIRCalibration-response>)))
  "Returns full string definition for message of type '<SetIRCalibration-response>"
  (cl:format cl:nil "bool success   # indicate successful run of triggered service~%string message # informational, e.g. for error messages~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetIRCalibration-response)))
  "Returns full string definition for message of type 'SetIRCalibration-response"
  (cl:format cl:nil "bool success   # indicate successful run of triggered service~%string message # informational, e.g. for error messages~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetIRCalibration-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetIRCalibration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetIRCalibration-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetIRCalibration)))
  'SetIRCalibration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetIRCalibration)))
  'SetIRCalibration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIRCalibration)))
  "Returns string type for a service object of type '<SetIRCalibration>"
  "hero_common/SetIRCalibration")