; Auto-generated. Do not edit!


(cl:in-package ur3_move-srv)


;//! \htmlinclude Stop-request.msg.html

(cl:defclass <Stop-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:integer
    :initform 0))
)

(cl:defclass Stop-request (<Stop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur3_move-srv:<Stop-request> is deprecated: use ur3_move-srv:Stop-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <Stop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:command-val is deprecated.  Use ur3_move-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stop-request>) ostream)
  "Serializes a message object of type '<Stop-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'command)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'command)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stop-request>) istream)
  "Deserializes a message object of type '<Stop-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'command)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stop-request>)))
  "Returns string type for a service object of type '<Stop-request>"
  "ur3_move/StopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stop-request)))
  "Returns string type for a service object of type 'Stop-request"
  "ur3_move/StopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stop-request>)))
  "Returns md5sum for a message object of type '<Stop-request>"
  "afc6579bcf2451c763cb3bef58337e2b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stop-request)))
  "Returns md5sum for a message object of type 'Stop-request"
  "afc6579bcf2451c763cb3bef58337e2b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stop-request>)))
  "Returns full string definition for message of type '<Stop-request>"
  (cl:format cl:nil "uint32 command~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stop-request)))
  "Returns full string definition for message of type 'Stop-request"
  (cl:format cl:nil "uint32 command~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stop-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Stop-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude Stop-response.msg.html

(cl:defclass <Stop-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Stop-response (<Stop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur3_move-srv:<Stop-response> is deprecated: use ur3_move-srv:Stop-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Stop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:result-val is deprecated.  Use ur3_move-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stop-response>) ostream)
  "Serializes a message object of type '<Stop-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stop-response>) istream)
  "Deserializes a message object of type '<Stop-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stop-response>)))
  "Returns string type for a service object of type '<Stop-response>"
  "ur3_move/StopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stop-response)))
  "Returns string type for a service object of type 'Stop-response"
  "ur3_move/StopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stop-response>)))
  "Returns md5sum for a message object of type '<Stop-response>"
  "afc6579bcf2451c763cb3bef58337e2b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stop-response)))
  "Returns md5sum for a message object of type 'Stop-response"
  "afc6579bcf2451c763cb3bef58337e2b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stop-response>)))
  "Returns full string definition for message of type '<Stop-response>"
  (cl:format cl:nil "~%bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stop-response)))
  "Returns full string definition for message of type 'Stop-response"
  (cl:format cl:nil "~%bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stop-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Stop-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Stop)))
  'Stop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Stop)))
  'Stop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stop)))
  "Returns string type for a service object of type '<Stop>"
  "ur3_move/Stop")