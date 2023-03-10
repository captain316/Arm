; Auto-generated. Do not edit!


(cl:in-package ur3_move-srv)


;//! \htmlinclude varObjectsPosition-request.msg.html

(cl:defclass <varObjectsPosition-request> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass varObjectsPosition-request (<varObjectsPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <varObjectsPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'varObjectsPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur3_move-srv:<varObjectsPosition-request> is deprecated: use ur3_move-srv:varObjectsPosition-request instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <varObjectsPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:flag-val is deprecated.  Use ur3_move-srv:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <varObjectsPosition-request>) ostream)
  "Serializes a message object of type '<varObjectsPosition-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <varObjectsPosition-request>) istream)
  "Deserializes a message object of type '<varObjectsPosition-request>"
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<varObjectsPosition-request>)))
  "Returns string type for a service object of type '<varObjectsPosition-request>"
  "ur3_move/varObjectsPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'varObjectsPosition-request)))
  "Returns string type for a service object of type 'varObjectsPosition-request"
  "ur3_move/varObjectsPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<varObjectsPosition-request>)))
  "Returns md5sum for a message object of type '<varObjectsPosition-request>"
  "471fff188bc2b994ff747dcd01944fb6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'varObjectsPosition-request)))
  "Returns md5sum for a message object of type 'varObjectsPosition-request"
  "471fff188bc2b994ff747dcd01944fb6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<varObjectsPosition-request>)))
  "Returns full string definition for message of type '<varObjectsPosition-request>"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'varObjectsPosition-request)))
  "Returns full string definition for message of type 'varObjectsPosition-request"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <varObjectsPosition-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <varObjectsPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'varObjectsPosition-request
    (cl:cons ':flag (flag msg))
))
;//! \htmlinclude varObjectsPosition-response.msg.html

(cl:defclass <varObjectsPosition-response> (roslisp-msg-protocol:ros-message)
  ((targets_pose
    :reader targets_pose
    :initarg :targets_pose
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (angles
    :reader angles
    :initarg :angles
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass varObjectsPosition-response (<varObjectsPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <varObjectsPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'varObjectsPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur3_move-srv:<varObjectsPosition-response> is deprecated: use ur3_move-srv:varObjectsPosition-response instead.")))

(cl:ensure-generic-function 'targets_pose-val :lambda-list '(m))
(cl:defmethod targets_pose-val ((m <varObjectsPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:targets_pose-val is deprecated.  Use ur3_move-srv:targets_pose instead.")
  (targets_pose m))

(cl:ensure-generic-function 'angles-val :lambda-list '(m))
(cl:defmethod angles-val ((m <varObjectsPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:angles-val is deprecated.  Use ur3_move-srv:angles instead.")
  (angles m))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <varObjectsPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:names-val is deprecated.  Use ur3_move-srv:names instead.")
  (names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <varObjectsPosition-response>) ostream)
  "Serializes a message object of type '<varObjectsPosition-response>"
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'names))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <varObjectsPosition-response>) istream)
  "Deserializes a message object of type '<varObjectsPosition-response>"
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<varObjectsPosition-response>)))
  "Returns string type for a service object of type '<varObjectsPosition-response>"
  "ur3_move/varObjectsPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'varObjectsPosition-response)))
  "Returns string type for a service object of type 'varObjectsPosition-response"
  "ur3_move/varObjectsPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<varObjectsPosition-response>)))
  "Returns md5sum for a message object of type '<varObjectsPosition-response>"
  "471fff188bc2b994ff747dcd01944fb6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'varObjectsPosition-response)))
  "Returns md5sum for a message object of type 'varObjectsPosition-response"
  "471fff188bc2b994ff747dcd01944fb6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<varObjectsPosition-response>)))
  "Returns full string definition for message of type '<varObjectsPosition-response>"
  (cl:format cl:nil "geometry_msgs/Pose[] targets_pose~%float64[] angles~%string[] names~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'varObjectsPosition-response)))
  "Returns full string definition for message of type 'varObjectsPosition-response"
  (cl:format cl:nil "geometry_msgs/Pose[] targets_pose~%float64[] angles~%string[] names~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <varObjectsPosition-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'targets_pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'angles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <varObjectsPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'varObjectsPosition-response
    (cl:cons ':targets_pose (targets_pose msg))
    (cl:cons ':angles (angles msg))
    (cl:cons ':names (names msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'varObjectsPosition)))
  'varObjectsPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'varObjectsPosition)))
  'varObjectsPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'varObjectsPosition)))
  "Returns string type for a service object of type '<varObjectsPosition>"
  "ur3_move/varObjectsPosition")