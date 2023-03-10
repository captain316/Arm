; Auto-generated. Do not edit!


(cl:in-package gohome-srv)


;//! \htmlinclude carArm-request.msg.html

(cl:defclass <carArm-request> (roslisp-msg-protocol:ros-message)
  ((location
    :reader location
    :initarg :location
    :type cl:string
    :initform ""))
)

(cl:defclass carArm-request (<carArm-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <carArm-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'carArm-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gohome-srv:<carArm-request> is deprecated: use gohome-srv:carArm-request instead.")))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <carArm-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gohome-srv:location-val is deprecated.  Use gohome-srv:location instead.")
  (location m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <carArm-request>) ostream)
  "Serializes a message object of type '<carArm-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'location))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'location))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <carArm-request>) istream)
  "Deserializes a message object of type '<carArm-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'location) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'location) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<carArm-request>)))
  "Returns string type for a service object of type '<carArm-request>"
  "gohome/carArmRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'carArm-request)))
  "Returns string type for a service object of type 'carArm-request"
  "gohome/carArmRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<carArm-request>)))
  "Returns md5sum for a message object of type '<carArm-request>"
  "f12396c1cf3ed85338aaa27ae2758520")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'carArm-request)))
  "Returns md5sum for a message object of type 'carArm-request"
  "f12396c1cf3ed85338aaa27ae2758520")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<carArm-request>)))
  "Returns full string definition for message of type '<carArm-request>"
  (cl:format cl:nil "string location~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'carArm-request)))
  "Returns full string definition for message of type 'carArm-request"
  (cl:format cl:nil "string location~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <carArm-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'location))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <carArm-request>))
  "Converts a ROS message object to a list"
  (cl:list 'carArm-request
    (cl:cons ':location (location msg))
))
;//! \htmlinclude carArm-response.msg.html

(cl:defclass <carArm-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass carArm-response (<carArm-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <carArm-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'carArm-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gohome-srv:<carArm-response> is deprecated: use gohome-srv:carArm-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <carArm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gohome-srv:result-val is deprecated.  Use gohome-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <carArm-response>) ostream)
  "Serializes a message object of type '<carArm-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <carArm-response>) istream)
  "Deserializes a message object of type '<carArm-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<carArm-response>)))
  "Returns string type for a service object of type '<carArm-response>"
  "gohome/carArmResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'carArm-response)))
  "Returns string type for a service object of type 'carArm-response"
  "gohome/carArmResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<carArm-response>)))
  "Returns md5sum for a message object of type '<carArm-response>"
  "f12396c1cf3ed85338aaa27ae2758520")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'carArm-response)))
  "Returns md5sum for a message object of type 'carArm-response"
  "f12396c1cf3ed85338aaa27ae2758520")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<carArm-response>)))
  "Returns full string definition for message of type '<carArm-response>"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'carArm-response)))
  "Returns full string definition for message of type 'carArm-response"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <carArm-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <carArm-response>))
  "Converts a ROS message object to a list"
  (cl:list 'carArm-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'carArm)))
  'carArm-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'carArm)))
  'carArm-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'carArm)))
  "Returns string type for a service object of type '<carArm>"
  "gohome/carArm")