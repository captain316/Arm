; Auto-generated. Do not edit!


(cl:in-package ur3_move-srv)


;//! \htmlinclude mulObjectsPosition-request.msg.html

(cl:defclass <mulObjectsPosition-request> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass mulObjectsPosition-request (<mulObjectsPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mulObjectsPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mulObjectsPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur3_move-srv:<mulObjectsPosition-request> is deprecated: use ur3_move-srv:mulObjectsPosition-request instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <mulObjectsPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:flag-val is deprecated.  Use ur3_move-srv:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mulObjectsPosition-request>) ostream)
  "Serializes a message object of type '<mulObjectsPosition-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mulObjectsPosition-request>) istream)
  "Deserializes a message object of type '<mulObjectsPosition-request>"
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mulObjectsPosition-request>)))
  "Returns string type for a service object of type '<mulObjectsPosition-request>"
  "ur3_move/mulObjectsPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mulObjectsPosition-request)))
  "Returns string type for a service object of type 'mulObjectsPosition-request"
  "ur3_move/mulObjectsPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mulObjectsPosition-request>)))
  "Returns md5sum for a message object of type '<mulObjectsPosition-request>"
  "c1c1fd25f8a31070cac107754a1daacf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mulObjectsPosition-request)))
  "Returns md5sum for a message object of type 'mulObjectsPosition-request"
  "c1c1fd25f8a31070cac107754a1daacf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mulObjectsPosition-request>)))
  "Returns full string definition for message of type '<mulObjectsPosition-request>"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mulObjectsPosition-request)))
  "Returns full string definition for message of type 'mulObjectsPosition-request"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mulObjectsPosition-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mulObjectsPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'mulObjectsPosition-request
    (cl:cons ':flag (flag msg))
))
;//! \htmlinclude mulObjectsPosition-response.msg.html

(cl:defclass <mulObjectsPosition-response> (roslisp-msg-protocol:ros-message)
  ((targets_pose
    :reader targets_pose
    :initarg :targets_pose
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (angles
    :reader angles
    :initarg :angles
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass mulObjectsPosition-response (<mulObjectsPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mulObjectsPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mulObjectsPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur3_move-srv:<mulObjectsPosition-response> is deprecated: use ur3_move-srv:mulObjectsPosition-response instead.")))

(cl:ensure-generic-function 'targets_pose-val :lambda-list '(m))
(cl:defmethod targets_pose-val ((m <mulObjectsPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:targets_pose-val is deprecated.  Use ur3_move-srv:targets_pose instead.")
  (targets_pose m))

(cl:ensure-generic-function 'angles-val :lambda-list '(m))
(cl:defmethod angles-val ((m <mulObjectsPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:angles-val is deprecated.  Use ur3_move-srv:angles instead.")
  (angles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mulObjectsPosition-response>) ostream)
  "Serializes a message object of type '<mulObjectsPosition-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'targets_pose))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'targets_pose))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'angles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'angles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mulObjectsPosition-response>) istream)
  "Deserializes a message object of type '<mulObjectsPosition-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'targets_pose) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'targets_pose)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'angles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'angles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mulObjectsPosition-response>)))
  "Returns string type for a service object of type '<mulObjectsPosition-response>"
  "ur3_move/mulObjectsPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mulObjectsPosition-response)))
  "Returns string type for a service object of type 'mulObjectsPosition-response"
  "ur3_move/mulObjectsPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mulObjectsPosition-response>)))
  "Returns md5sum for a message object of type '<mulObjectsPosition-response>"
  "c1c1fd25f8a31070cac107754a1daacf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mulObjectsPosition-response)))
  "Returns md5sum for a message object of type 'mulObjectsPosition-response"
  "c1c1fd25f8a31070cac107754a1daacf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mulObjectsPosition-response>)))
  "Returns full string definition for message of type '<mulObjectsPosition-response>"
  (cl:format cl:nil "geometry_msgs/Pose[] targets_pose~%float64[] angles~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mulObjectsPosition-response)))
  "Returns full string definition for message of type 'mulObjectsPosition-response"
  (cl:format cl:nil "geometry_msgs/Pose[] targets_pose~%float64[] angles~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mulObjectsPosition-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'targets_pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'angles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mulObjectsPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'mulObjectsPosition-response
    (cl:cons ':targets_pose (targets_pose msg))
    (cl:cons ':angles (angles msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'mulObjectsPosition)))
  'mulObjectsPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'mulObjectsPosition)))
  'mulObjectsPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mulObjectsPosition)))
  "Returns string type for a service object of type '<mulObjectsPosition>"
  "ur3_move/mulObjectsPosition")