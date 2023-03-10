; Auto-generated. Do not edit!


(cl:in-package ur3_move-srv)


;//! \htmlinclude getObjectPosition-request.msg.html

(cl:defclass <getObjectPosition-request> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass getObjectPosition-request (<getObjectPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getObjectPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getObjectPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur3_move-srv:<getObjectPosition-request> is deprecated: use ur3_move-srv:getObjectPosition-request instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <getObjectPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:flag-val is deprecated.  Use ur3_move-srv:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getObjectPosition-request>) ostream)
  "Serializes a message object of type '<getObjectPosition-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getObjectPosition-request>) istream)
  "Deserializes a message object of type '<getObjectPosition-request>"
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getObjectPosition-request>)))
  "Returns string type for a service object of type '<getObjectPosition-request>"
  "ur3_move/getObjectPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getObjectPosition-request)))
  "Returns string type for a service object of type 'getObjectPosition-request"
  "ur3_move/getObjectPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getObjectPosition-request>)))
  "Returns md5sum for a message object of type '<getObjectPosition-request>"
  "914a1e14d98b3c79e57093c7bd6ee205")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getObjectPosition-request)))
  "Returns md5sum for a message object of type 'getObjectPosition-request"
  "914a1e14d98b3c79e57093c7bd6ee205")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getObjectPosition-request>)))
  "Returns full string definition for message of type '<getObjectPosition-request>"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getObjectPosition-request)))
  "Returns full string definition for message of type 'getObjectPosition-request"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getObjectPosition-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getObjectPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getObjectPosition-request
    (cl:cons ':flag (flag msg))
))
;//! \htmlinclude getObjectPosition-response.msg.html

(cl:defclass <getObjectPosition-response> (roslisp-msg-protocol:ros-message)
  ((target_pose
    :reader target_pose
    :initarg :target_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass getObjectPosition-response (<getObjectPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getObjectPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getObjectPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur3_move-srv:<getObjectPosition-response> is deprecated: use ur3_move-srv:getObjectPosition-response instead.")))

(cl:ensure-generic-function 'target_pose-val :lambda-list '(m))
(cl:defmethod target_pose-val ((m <getObjectPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:target_pose-val is deprecated.  Use ur3_move-srv:target_pose instead.")
  (target_pose m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <getObjectPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:angle-val is deprecated.  Use ur3_move-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getObjectPosition-response>) ostream)
  "Serializes a message object of type '<getObjectPosition-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getObjectPosition-response>) istream)
  "Deserializes a message object of type '<getObjectPosition-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getObjectPosition-response>)))
  "Returns string type for a service object of type '<getObjectPosition-response>"
  "ur3_move/getObjectPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getObjectPosition-response)))
  "Returns string type for a service object of type 'getObjectPosition-response"
  "ur3_move/getObjectPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getObjectPosition-response>)))
  "Returns md5sum for a message object of type '<getObjectPosition-response>"
  "914a1e14d98b3c79e57093c7bd6ee205")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getObjectPosition-response)))
  "Returns md5sum for a message object of type 'getObjectPosition-response"
  "914a1e14d98b3c79e57093c7bd6ee205")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getObjectPosition-response>)))
  "Returns full string definition for message of type '<getObjectPosition-response>"
  (cl:format cl:nil "geometry_msgs/Pose target_pose~%float64 angle~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getObjectPosition-response)))
  "Returns full string definition for message of type 'getObjectPosition-response"
  (cl:format cl:nil "geometry_msgs/Pose target_pose~%float64 angle~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getObjectPosition-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_pose))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getObjectPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getObjectPosition-response
    (cl:cons ':target_pose (target_pose msg))
    (cl:cons ':angle (angle msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getObjectPosition)))
  'getObjectPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getObjectPosition)))
  'getObjectPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getObjectPosition)))
  "Returns string type for a service object of type '<getObjectPosition>"
  "ur3_move/getObjectPosition")