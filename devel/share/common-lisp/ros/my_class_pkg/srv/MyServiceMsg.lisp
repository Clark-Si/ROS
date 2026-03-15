; Auto-generated. Do not edit!


(cl:in-package my_class_pkg-srv)


;//! \htmlinclude MyServiceMsg-request.msg.html

(cl:defclass <MyServiceMsg-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type cl:integer
    :initform 0))
)

(cl:defclass MyServiceMsg-request (<MyServiceMsg-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MyServiceMsg-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MyServiceMsg-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_class_pkg-srv:<MyServiceMsg-request> is deprecated: use my_class_pkg-srv:MyServiceMsg-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <MyServiceMsg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_class_pkg-srv:input-val is deprecated.  Use my_class_pkg-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MyServiceMsg-request>) ostream)
  "Serializes a message object of type '<MyServiceMsg-request>"
  (cl:let* ((signed (cl:slot-value msg 'input)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MyServiceMsg-request>) istream)
  "Deserializes a message object of type '<MyServiceMsg-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'input) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MyServiceMsg-request>)))
  "Returns string type for a service object of type '<MyServiceMsg-request>"
  "my_class_pkg/MyServiceMsgRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MyServiceMsg-request)))
  "Returns string type for a service object of type 'MyServiceMsg-request"
  "my_class_pkg/MyServiceMsgRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MyServiceMsg-request>)))
  "Returns md5sum for a message object of type '<MyServiceMsg-request>"
  "9cc62145209219f185101e72e3ae5c65")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MyServiceMsg-request)))
  "Returns md5sum for a message object of type 'MyServiceMsg-request"
  "9cc62145209219f185101e72e3ae5c65")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MyServiceMsg-request>)))
  "Returns full string definition for message of type '<MyServiceMsg-request>"
  (cl:format cl:nil "int64 input # 查询~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MyServiceMsg-request)))
  "Returns full string definition for message of type 'MyServiceMsg-request"
  (cl:format cl:nil "int64 input # 查询~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MyServiceMsg-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MyServiceMsg-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MyServiceMsg-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude MyServiceMsg-response.msg.html

(cl:defclass <MyServiceMsg-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:integer
    :initform 0))
)

(cl:defclass MyServiceMsg-response (<MyServiceMsg-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MyServiceMsg-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MyServiceMsg-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_class_pkg-srv:<MyServiceMsg-response> is deprecated: use my_class_pkg-srv:MyServiceMsg-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <MyServiceMsg-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_class_pkg-srv:output-val is deprecated.  Use my_class_pkg-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MyServiceMsg-response>) ostream)
  "Serializes a message object of type '<MyServiceMsg-response>"
  (cl:let* ((signed (cl:slot-value msg 'output)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MyServiceMsg-response>) istream)
  "Deserializes a message object of type '<MyServiceMsg-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'output) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MyServiceMsg-response>)))
  "Returns string type for a service object of type '<MyServiceMsg-response>"
  "my_class_pkg/MyServiceMsgResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MyServiceMsg-response)))
  "Returns string type for a service object of type 'MyServiceMsg-response"
  "my_class_pkg/MyServiceMsgResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MyServiceMsg-response>)))
  "Returns md5sum for a message object of type '<MyServiceMsg-response>"
  "9cc62145209219f185101e72e3ae5c65")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MyServiceMsg-response)))
  "Returns md5sum for a message object of type 'MyServiceMsg-response"
  "9cc62145209219f185101e72e3ae5c65")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MyServiceMsg-response>)))
  "Returns full string definition for message of type '<MyServiceMsg-response>"
  (cl:format cl:nil "int64 output # 回复~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MyServiceMsg-response)))
  "Returns full string definition for message of type 'MyServiceMsg-response"
  (cl:format cl:nil "int64 output # 回复~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MyServiceMsg-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MyServiceMsg-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MyServiceMsg-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MyServiceMsg)))
  'MyServiceMsg-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MyServiceMsg)))
  'MyServiceMsg-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MyServiceMsg)))
  "Returns string type for a service object of type '<MyServiceMsg>"
  "my_class_pkg/MyServiceMsg")