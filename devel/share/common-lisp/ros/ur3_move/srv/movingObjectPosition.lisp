; Auto-generated. Do not edit!


(cl:in-package ur3_move-srv)


;//! \htmlinclude movingObjectPosition-request.msg.html

(cl:defclass <movingObjectPosition-request> (roslisp-msg-protocol:ros-message)
  ((send
    :reader send
    :initarg :send
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass movingObjectPosition-request (<movingObjectPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <movingObjectPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'movingObjectPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur3_move-srv:<movingObjectPosition-request> is deprecated: use ur3_move-srv:movingObjectPosition-request instead.")))

(cl:ensure-generic-function 'send-val :lambda-list '(m))
(cl:defmethod send-val ((m <movingObjectPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:send-val is deprecated.  Use ur3_move-srv:send instead.")
  (send m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <movingObjectPosition-request>) ostream)
  "Serializes a message object of type '<movingObjectPosition-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'send) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <movingObjectPosition-request>) istream)
  "Deserializes a message object of type '<movingObjectPosition-request>"
    (cl:setf (cl:slot-value msg 'send) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<movingObjectPosition-request>)))
  "Returns string type for a service object of type '<movingObjectPosition-request>"
  "ur3_move/movingObjectPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'movingObjectPosition-request)))
  "Returns string type for a service object of type 'movingObjectPosition-request"
  "ur3_move/movingObjectPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<movingObjectPosition-request>)))
  "Returns md5sum for a message object of type '<movingObjectPosition-request>"
  "05475c87257688fa5ef65958faf092d7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'movingObjectPosition-request)))
  "Returns md5sum for a message object of type 'movingObjectPosition-request"
  "05475c87257688fa5ef65958faf092d7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<movingObjectPosition-request>)))
  "Returns full string definition for message of type '<movingObjectPosition-request>"
  (cl:format cl:nil "bool send~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'movingObjectPosition-request)))
  "Returns full string definition for message of type 'movingObjectPosition-request"
  (cl:format cl:nil "bool send~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <movingObjectPosition-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <movingObjectPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'movingObjectPosition-request
    (cl:cons ':send (send msg))
))
;//! \htmlinclude movingObjectPosition-response.msg.html

(cl:defclass <movingObjectPosition-response> (roslisp-msg-protocol:ros-message)
  ((receive
    :reader receive
    :initarg :receive
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass movingObjectPosition-response (<movingObjectPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <movingObjectPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'movingObjectPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur3_move-srv:<movingObjectPosition-response> is deprecated: use ur3_move-srv:movingObjectPosition-response instead.")))

(cl:ensure-generic-function 'receive-val :lambda-list '(m))
(cl:defmethod receive-val ((m <movingObjectPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_move-srv:receive-val is deprecated.  Use ur3_move-srv:receive instead.")
  (receive m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <movingObjectPosition-response>) ostream)
  "Serializes a message object of type '<movingObjectPosition-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'receive) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <movingObjectPosition-response>) istream)
  "Deserializes a message object of type '<movingObjectPosition-response>"
    (cl:setf (cl:slot-value msg 'receive) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<movingObjectPosition-response>)))
  "Returns string type for a service object of type '<movingObjectPosition-response>"
  "ur3_move/movingObjectPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'movingObjectPosition-response)))
  "Returns string type for a service object of type 'movingObjectPosition-response"
  "ur3_move/movingObjectPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<movingObjectPosition-response>)))
  "Returns md5sum for a message object of type '<movingObjectPosition-response>"
  "05475c87257688fa5ef65958faf092d7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'movingObjectPosition-response)))
  "Returns md5sum for a message object of type 'movingObjectPosition-response"
  "05475c87257688fa5ef65958faf092d7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<movingObjectPosition-response>)))
  "Returns full string definition for message of type '<movingObjectPosition-response>"
  (cl:format cl:nil "bool receive~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'movingObjectPosition-response)))
  "Returns full string definition for message of type 'movingObjectPosition-response"
  (cl:format cl:nil "bool receive~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <movingObjectPosition-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <movingObjectPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'movingObjectPosition-response
    (cl:cons ':receive (receive msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'movingObjectPosition)))
  'movingObjectPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'movingObjectPosition)))
  'movingObjectPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'movingObjectPosition)))
  "Returns string type for a service object of type '<movingObjectPosition>"
  "ur3_move/movingObjectPosition")