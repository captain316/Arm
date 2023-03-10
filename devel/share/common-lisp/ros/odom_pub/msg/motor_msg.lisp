; Auto-generated. Do not edit!


(cl:in-package odom_pub-msg)


;//! \htmlinclude motor_msg.msg.html

(cl:defclass <motor_msg> (roslisp-msg-protocol:ros-message)
  ((driver_motor_01
    :reader driver_motor_01
    :initarg :driver_motor_01
    :type cl:float
    :initform 0.0)
   (driver_motor_02
    :reader driver_motor_02
    :initarg :driver_motor_02
    :type cl:float
    :initform 0.0)
   (driver_motor_03
    :reader driver_motor_03
    :initarg :driver_motor_03
    :type cl:float
    :initform 0.0)
   (driver_motor_04
    :reader driver_motor_04
    :initarg :driver_motor_04
    :type cl:float
    :initform 0.0)
   (steer_motor_01
    :reader steer_motor_01
    :initarg :steer_motor_01
    :type cl:float
    :initform 0.0)
   (steer_motor_02
    :reader steer_motor_02
    :initarg :steer_motor_02
    :type cl:float
    :initform 0.0)
   (steer_motor_03
    :reader steer_motor_03
    :initarg :steer_motor_03
    :type cl:float
    :initform 0.0)
   (steer_motor_04
    :reader steer_motor_04
    :initarg :steer_motor_04
    :type cl:float
    :initform 0.0))
)

(cl:defclass motor_msg (<motor_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odom_pub-msg:<motor_msg> is deprecated: use odom_pub-msg:motor_msg instead.")))

(cl:ensure-generic-function 'driver_motor_01-val :lambda-list '(m))
(cl:defmethod driver_motor_01-val ((m <motor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odom_pub-msg:driver_motor_01-val is deprecated.  Use odom_pub-msg:driver_motor_01 instead.")
  (driver_motor_01 m))

(cl:ensure-generic-function 'driver_motor_02-val :lambda-list '(m))
(cl:defmethod driver_motor_02-val ((m <motor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odom_pub-msg:driver_motor_02-val is deprecated.  Use odom_pub-msg:driver_motor_02 instead.")
  (driver_motor_02 m))

(cl:ensure-generic-function 'driver_motor_03-val :lambda-list '(m))
(cl:defmethod driver_motor_03-val ((m <motor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odom_pub-msg:driver_motor_03-val is deprecated.  Use odom_pub-msg:driver_motor_03 instead.")
  (driver_motor_03 m))

(cl:ensure-generic-function 'driver_motor_04-val :lambda-list '(m))
(cl:defmethod driver_motor_04-val ((m <motor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odom_pub-msg:driver_motor_04-val is deprecated.  Use odom_pub-msg:driver_motor_04 instead.")
  (driver_motor_04 m))

(cl:ensure-generic-function 'steer_motor_01-val :lambda-list '(m))
(cl:defmethod steer_motor_01-val ((m <motor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odom_pub-msg:steer_motor_01-val is deprecated.  Use odom_pub-msg:steer_motor_01 instead.")
  (steer_motor_01 m))

(cl:ensure-generic-function 'steer_motor_02-val :lambda-list '(m))
(cl:defmethod steer_motor_02-val ((m <motor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odom_pub-msg:steer_motor_02-val is deprecated.  Use odom_pub-msg:steer_motor_02 instead.")
  (steer_motor_02 m))

(cl:ensure-generic-function 'steer_motor_03-val :lambda-list '(m))
(cl:defmethod steer_motor_03-val ((m <motor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odom_pub-msg:steer_motor_03-val is deprecated.  Use odom_pub-msg:steer_motor_03 instead.")
  (steer_motor_03 m))

(cl:ensure-generic-function 'steer_motor_04-val :lambda-list '(m))
(cl:defmethod steer_motor_04-val ((m <motor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odom_pub-msg:steer_motor_04-val is deprecated.  Use odom_pub-msg:steer_motor_04 instead.")
  (steer_motor_04 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_msg>) ostream)
  "Serializes a message object of type '<motor_msg>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'driver_motor_01))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'driver_motor_02))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'driver_motor_03))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'driver_motor_04))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steer_motor_01))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steer_motor_02))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steer_motor_03))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steer_motor_04))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_msg>) istream)
  "Deserializes a message object of type '<motor_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'driver_motor_01) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'driver_motor_02) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'driver_motor_03) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'driver_motor_04) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer_motor_01) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer_motor_02) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer_motor_03) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer_motor_04) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_msg>)))
  "Returns string type for a message object of type '<motor_msg>"
  "odom_pub/motor_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_msg)))
  "Returns string type for a message object of type 'motor_msg"
  "odom_pub/motor_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_msg>)))
  "Returns md5sum for a message object of type '<motor_msg>"
  "0e4c0fdbf8c8204c136aa0e02d2bf289")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_msg)))
  "Returns md5sum for a message object of type 'motor_msg"
  "0e4c0fdbf8c8204c136aa0e02d2bf289")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_msg>)))
  "Returns full string definition for message of type '<motor_msg>"
  (cl:format cl:nil "float64 driver_motor_01     ~%float64 driver_motor_02~%float64 driver_motor_03~%float64 driver_motor_04~%~%float64 steer_motor_01~%float64 steer_motor_02~%float64 steer_motor_03~%float64 steer_motor_04~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_msg)))
  "Returns full string definition for message of type 'motor_msg"
  (cl:format cl:nil "float64 driver_motor_01     ~%float64 driver_motor_02~%float64 driver_motor_03~%float64 driver_motor_04~%~%float64 steer_motor_01~%float64 steer_motor_02~%float64 steer_motor_03~%float64 steer_motor_04~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_msg>))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_msg
    (cl:cons ':driver_motor_01 (driver_motor_01 msg))
    (cl:cons ':driver_motor_02 (driver_motor_02 msg))
    (cl:cons ':driver_motor_03 (driver_motor_03 msg))
    (cl:cons ':driver_motor_04 (driver_motor_04 msg))
    (cl:cons ':steer_motor_01 (steer_motor_01 msg))
    (cl:cons ':steer_motor_02 (steer_motor_02 msg))
    (cl:cons ':steer_motor_03 (steer_motor_03 msg))
    (cl:cons ':steer_motor_04 (steer_motor_04 msg))
))
