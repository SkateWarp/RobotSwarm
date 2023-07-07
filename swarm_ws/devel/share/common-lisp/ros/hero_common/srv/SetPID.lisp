; Auto-generated. Do not edit!


(cl:in-package hero_common-srv)


;//! \htmlinclude SetPID-request.msg.html

(cl:defclass <SetPID-request> (roslisp-msg-protocol:ros-message)
  ((lkp
    :reader lkp
    :initarg :lkp
    :type cl:float
    :initform 0.0)
   (lki
    :reader lki
    :initarg :lki
    :type cl:float
    :initform 0.0)
   (lkd
    :reader lkd
    :initarg :lkd
    :type cl:float
    :initform 0.0)
   (rkp
    :reader rkp
    :initarg :rkp
    :type cl:float
    :initform 0.0)
   (rki
    :reader rki
    :initarg :rki
    :type cl:float
    :initform 0.0)
   (rkd
    :reader rkd
    :initarg :rkd
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetPID-request (<SetPID-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPID-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPID-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hero_common-srv:<SetPID-request> is deprecated: use hero_common-srv:SetPID-request instead.")))

(cl:ensure-generic-function 'lkp-val :lambda-list '(m))
(cl:defmethod lkp-val ((m <SetPID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:lkp-val is deprecated.  Use hero_common-srv:lkp instead.")
  (lkp m))

(cl:ensure-generic-function 'lki-val :lambda-list '(m))
(cl:defmethod lki-val ((m <SetPID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:lki-val is deprecated.  Use hero_common-srv:lki instead.")
  (lki m))

(cl:ensure-generic-function 'lkd-val :lambda-list '(m))
(cl:defmethod lkd-val ((m <SetPID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:lkd-val is deprecated.  Use hero_common-srv:lkd instead.")
  (lkd m))

(cl:ensure-generic-function 'rkp-val :lambda-list '(m))
(cl:defmethod rkp-val ((m <SetPID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:rkp-val is deprecated.  Use hero_common-srv:rkp instead.")
  (rkp m))

(cl:ensure-generic-function 'rki-val :lambda-list '(m))
(cl:defmethod rki-val ((m <SetPID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:rki-val is deprecated.  Use hero_common-srv:rki instead.")
  (rki m))

(cl:ensure-generic-function 'rkd-val :lambda-list '(m))
(cl:defmethod rkd-val ((m <SetPID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:rkd-val is deprecated.  Use hero_common-srv:rkd instead.")
  (rkd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPID-request>) ostream)
  "Serializes a message object of type '<SetPID-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lkp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lki))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lkd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rkp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rki))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rkd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPID-request>) istream)
  "Deserializes a message object of type '<SetPID-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lkp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lki) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lkd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rkp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rki) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rkd) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPID-request>)))
  "Returns string type for a service object of type '<SetPID-request>"
  "hero_common/SetPIDRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPID-request)))
  "Returns string type for a service object of type 'SetPID-request"
  "hero_common/SetPIDRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPID-request>)))
  "Returns md5sum for a message object of type '<SetPID-request>"
  "69c16eccc66fd6b477afb5e5500b4b47")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPID-request)))
  "Returns md5sum for a message object of type 'SetPID-request"
  "69c16eccc66fd6b477afb5e5500b4b47")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPID-request>)))
  "Returns full string definition for message of type '<SetPID-request>"
  (cl:format cl:nil "float32 lkp # proportional term left motor~%float32 lki # intergrative term left motor~%float32 lkd # derivative term left motor~%float32 rkp # proportional term right motor~%float32 rki # intergrative term right motor~%float32 rkd # derivative term right motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPID-request)))
  "Returns full string definition for message of type 'SetPID-request"
  (cl:format cl:nil "float32 lkp # proportional term left motor~%float32 lki # intergrative term left motor~%float32 lkd # derivative term left motor~%float32 rkp # proportional term right motor~%float32 rki # intergrative term right motor~%float32 rkd # derivative term right motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPID-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPID-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPID-request
    (cl:cons ':lkp (lkp msg))
    (cl:cons ':lki (lki msg))
    (cl:cons ':lkd (lkd msg))
    (cl:cons ':rkp (rkp msg))
    (cl:cons ':rki (rki msg))
    (cl:cons ':rkd (rkd msg))
))
;//! \htmlinclude SetPID-response.msg.html

(cl:defclass <SetPID-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetPID-response (<SetPID-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPID-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPID-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hero_common-srv:<SetPID-response> is deprecated: use hero_common-srv:SetPID-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetPID-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:success-val is deprecated.  Use hero_common-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetPID-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:message-val is deprecated.  Use hero_common-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPID-response>) ostream)
  "Serializes a message object of type '<SetPID-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPID-response>) istream)
  "Deserializes a message object of type '<SetPID-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPID-response>)))
  "Returns string type for a service object of type '<SetPID-response>"
  "hero_common/SetPIDResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPID-response)))
  "Returns string type for a service object of type 'SetPID-response"
  "hero_common/SetPIDResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPID-response>)))
  "Returns md5sum for a message object of type '<SetPID-response>"
  "69c16eccc66fd6b477afb5e5500b4b47")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPID-response)))
  "Returns md5sum for a message object of type 'SetPID-response"
  "69c16eccc66fd6b477afb5e5500b4b47")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPID-response>)))
  "Returns full string definition for message of type '<SetPID-response>"
  (cl:format cl:nil "bool success   # indicate successful run of triggered service~%string message # informational, e.g. for error messages~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPID-response)))
  "Returns full string definition for message of type 'SetPID-response"
  (cl:format cl:nil "bool success   # indicate successful run of triggered service~%string message # informational, e.g. for error messages~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPID-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPID-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPID-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPID)))
  'SetPID-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPID)))
  'SetPID-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPID)))
  "Returns string type for a service object of type '<SetPID>"
  "hero_common/SetPID")