; Auto-generated. Do not edit!


(cl:in-package hero_common-srv)


;//! \htmlinclude SetOdom-request.msg.html

(cl:defclass <SetOdom-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetOdom-request (<SetOdom-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOdom-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOdom-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hero_common-srv:<SetOdom-request> is deprecated: use hero_common-srv:SetOdom-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <SetOdom-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:x-val is deprecated.  Use hero_common-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <SetOdom-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:y-val is deprecated.  Use hero_common-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <SetOdom-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:theta-val is deprecated.  Use hero_common-srv:theta instead.")
  (theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOdom-request>) ostream)
  "Serializes a message object of type '<SetOdom-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOdom-request>) istream)
  "Deserializes a message object of type '<SetOdom-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOdom-request>)))
  "Returns string type for a service object of type '<SetOdom-request>"
  "hero_common/SetOdomRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOdom-request)))
  "Returns string type for a service object of type 'SetOdom-request"
  "hero_common/SetOdomRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOdom-request>)))
  "Returns md5sum for a message object of type '<SetOdom-request>"
  "4ed90569b1afd039b963e42a12637d2b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOdom-request)))
  "Returns md5sum for a message object of type 'SetOdom-request"
  "4ed90569b1afd039b963e42a12637d2b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOdom-request>)))
  "Returns full string definition for message of type '<SetOdom-request>"
  (cl:format cl:nil "float64 x # proportional term~%float64 y # intergrative term~%float64 theta # derivative term~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOdom-request)))
  "Returns full string definition for message of type 'SetOdom-request"
  (cl:format cl:nil "float64 x # proportional term~%float64 y # intergrative term~%float64 theta # derivative term~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOdom-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOdom-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOdom-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':theta (theta msg))
))
;//! \htmlinclude SetOdom-response.msg.html

(cl:defclass <SetOdom-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetOdom-response (<SetOdom-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOdom-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOdom-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hero_common-srv:<SetOdom-response> is deprecated: use hero_common-srv:SetOdom-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetOdom-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:success-val is deprecated.  Use hero_common-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetOdom-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:message-val is deprecated.  Use hero_common-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOdom-response>) ostream)
  "Serializes a message object of type '<SetOdom-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOdom-response>) istream)
  "Deserializes a message object of type '<SetOdom-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOdom-response>)))
  "Returns string type for a service object of type '<SetOdom-response>"
  "hero_common/SetOdomResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOdom-response)))
  "Returns string type for a service object of type 'SetOdom-response"
  "hero_common/SetOdomResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOdom-response>)))
  "Returns md5sum for a message object of type '<SetOdom-response>"
  "4ed90569b1afd039b963e42a12637d2b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOdom-response)))
  "Returns md5sum for a message object of type 'SetOdom-response"
  "4ed90569b1afd039b963e42a12637d2b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOdom-response>)))
  "Returns full string definition for message of type '<SetOdom-response>"
  (cl:format cl:nil "bool success   # indicate successful run of triggered service~%string message # informational, e.g. for error messages~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOdom-response)))
  "Returns full string definition for message of type 'SetOdom-response"
  (cl:format cl:nil "bool success   # indicate successful run of triggered service~%string message # informational, e.g. for error messages~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOdom-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOdom-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOdom-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetOdom)))
  'SetOdom-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetOdom)))
  'SetOdom-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOdom)))
  "Returns string type for a service object of type '<SetOdom>"
  "hero_common/SetOdom")