; Auto-generated. Do not edit!


(cl:in-package hero_common-srv)


;//! \htmlinclude SetFrequency-request.msg.html

(cl:defclass <SetFrequency-request> (roslisp-msg-protocol:ros-message)
  ((frequency
    :reader frequency
    :initarg :frequency
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetFrequency-request (<SetFrequency-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFrequency-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFrequency-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hero_common-srv:<SetFrequency-request> is deprecated: use hero_common-srv:SetFrequency-request instead.")))

(cl:ensure-generic-function 'frequency-val :lambda-list '(m))
(cl:defmethod frequency-val ((m <SetFrequency-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:frequency-val is deprecated.  Use hero_common-srv:frequency instead.")
  (frequency m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFrequency-request>) ostream)
  "Serializes a message object of type '<SetFrequency-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFrequency-request>) istream)
  "Deserializes a message object of type '<SetFrequency-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frequency) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFrequency-request>)))
  "Returns string type for a service object of type '<SetFrequency-request>"
  "hero_common/SetFrequencyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFrequency-request)))
  "Returns string type for a service object of type 'SetFrequency-request"
  "hero_common/SetFrequencyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFrequency-request>)))
  "Returns md5sum for a message object of type '<SetFrequency-request>"
  "ff3ae8a6187feadac85089456faed43f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFrequency-request)))
  "Returns md5sum for a message object of type 'SetFrequency-request"
  "ff3ae8a6187feadac85089456faed43f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFrequency-request>)))
  "Returns full string definition for message of type '<SetFrequency-request>"
  (cl:format cl:nil "float64 frequency # loop-frequency in Hz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFrequency-request)))
  "Returns full string definition for message of type 'SetFrequency-request"
  (cl:format cl:nil "float64 frequency # loop-frequency in Hz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFrequency-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFrequency-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFrequency-request
    (cl:cons ':frequency (frequency msg))
))
;//! \htmlinclude SetFrequency-response.msg.html

(cl:defclass <SetFrequency-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetFrequency-response (<SetFrequency-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFrequency-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFrequency-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hero_common-srv:<SetFrequency-response> is deprecated: use hero_common-srv:SetFrequency-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetFrequency-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:success-val is deprecated.  Use hero_common-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetFrequency-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:message-val is deprecated.  Use hero_common-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFrequency-response>) ostream)
  "Serializes a message object of type '<SetFrequency-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFrequency-response>) istream)
  "Deserializes a message object of type '<SetFrequency-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFrequency-response>)))
  "Returns string type for a service object of type '<SetFrequency-response>"
  "hero_common/SetFrequencyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFrequency-response)))
  "Returns string type for a service object of type 'SetFrequency-response"
  "hero_common/SetFrequencyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFrequency-response>)))
  "Returns md5sum for a message object of type '<SetFrequency-response>"
  "ff3ae8a6187feadac85089456faed43f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFrequency-response)))
  "Returns md5sum for a message object of type 'SetFrequency-response"
  "ff3ae8a6187feadac85089456faed43f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFrequency-response>)))
  "Returns full string definition for message of type '<SetFrequency-response>"
  (cl:format cl:nil "bool success   # indicate successful run of triggered service~%string message # informational, e.g. for error messages~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFrequency-response)))
  "Returns full string definition for message of type 'SetFrequency-response"
  (cl:format cl:nil "bool success   # indicate successful run of triggered service~%string message # informational, e.g. for error messages~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFrequency-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFrequency-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFrequency-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetFrequency)))
  'SetFrequency-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetFrequency)))
  'SetFrequency-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFrequency)))
  "Returns string type for a service object of type '<SetFrequency>"
  "hero_common/SetFrequency")