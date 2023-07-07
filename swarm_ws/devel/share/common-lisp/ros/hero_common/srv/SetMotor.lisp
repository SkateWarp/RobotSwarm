; Auto-generated. Do not edit!


(cl:in-package hero_common-srv)


;//! \htmlinclude SetMotor-request.msg.html

(cl:defclass <SetMotor-request> (roslisp-msg-protocol:ros-message)
  ((left_motor_pwm
    :reader left_motor_pwm
    :initarg :left_motor_pwm
    :type cl:fixnum
    :initform 0)
   (right_motor_pwm
    :reader right_motor_pwm
    :initarg :right_motor_pwm
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetMotor-request (<SetMotor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMotor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMotor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hero_common-srv:<SetMotor-request> is deprecated: use hero_common-srv:SetMotor-request instead.")))

(cl:ensure-generic-function 'left_motor_pwm-val :lambda-list '(m))
(cl:defmethod left_motor_pwm-val ((m <SetMotor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:left_motor_pwm-val is deprecated.  Use hero_common-srv:left_motor_pwm instead.")
  (left_motor_pwm m))

(cl:ensure-generic-function 'right_motor_pwm-val :lambda-list '(m))
(cl:defmethod right_motor_pwm-val ((m <SetMotor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:right_motor_pwm-val is deprecated.  Use hero_common-srv:right_motor_pwm instead.")
  (right_motor_pwm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMotor-request>) ostream)
  "Serializes a message object of type '<SetMotor-request>"
  (cl:let* ((signed (cl:slot-value msg 'left_motor_pwm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_motor_pwm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMotor-request>) istream)
  "Deserializes a message object of type '<SetMotor-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_motor_pwm) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_motor_pwm) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMotor-request>)))
  "Returns string type for a service object of type '<SetMotor-request>"
  "hero_common/SetMotorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMotor-request)))
  "Returns string type for a service object of type 'SetMotor-request"
  "hero_common/SetMotorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMotor-request>)))
  "Returns md5sum for a message object of type '<SetMotor-request>"
  "07e32bced0fc14456815cdd097819b89")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMotor-request)))
  "Returns md5sum for a message object of type 'SetMotor-request"
  "07e32bced0fc14456815cdd097819b89")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMotor-request>)))
  "Returns full string definition for message of type '<SetMotor-request>"
  (cl:format cl:nil "int16 left_motor_pwm # motor deadzone pwm~%int16 right_motor_pwm # motor deadzone pwm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMotor-request)))
  "Returns full string definition for message of type 'SetMotor-request"
  (cl:format cl:nil "int16 left_motor_pwm # motor deadzone pwm~%int16 right_motor_pwm # motor deadzone pwm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMotor-request>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMotor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMotor-request
    (cl:cons ':left_motor_pwm (left_motor_pwm msg))
    (cl:cons ':right_motor_pwm (right_motor_pwm msg))
))
;//! \htmlinclude SetMotor-response.msg.html

(cl:defclass <SetMotor-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetMotor-response (<SetMotor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMotor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMotor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hero_common-srv:<SetMotor-response> is deprecated: use hero_common-srv:SetMotor-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetMotor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:success-val is deprecated.  Use hero_common-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetMotor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hero_common-srv:message-val is deprecated.  Use hero_common-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMotor-response>) ostream)
  "Serializes a message object of type '<SetMotor-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMotor-response>) istream)
  "Deserializes a message object of type '<SetMotor-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMotor-response>)))
  "Returns string type for a service object of type '<SetMotor-response>"
  "hero_common/SetMotorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMotor-response)))
  "Returns string type for a service object of type 'SetMotor-response"
  "hero_common/SetMotorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMotor-response>)))
  "Returns md5sum for a message object of type '<SetMotor-response>"
  "07e32bced0fc14456815cdd097819b89")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMotor-response)))
  "Returns md5sum for a message object of type 'SetMotor-response"
  "07e32bced0fc14456815cdd097819b89")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMotor-response>)))
  "Returns full string definition for message of type '<SetMotor-response>"
  (cl:format cl:nil "bool success   # indicate successful run of triggered service~%string message # informational, e.g. for error messages~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMotor-response)))
  "Returns full string definition for message of type 'SetMotor-response"
  (cl:format cl:nil "bool success   # indicate successful run of triggered service~%string message # informational, e.g. for error messages~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMotor-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMotor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMotor-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetMotor)))
  'SetMotor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetMotor)))
  'SetMotor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMotor)))
  "Returns string type for a service object of type '<SetMotor>"
  "hero_common/SetMotor")