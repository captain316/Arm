;; Auto-generated. Do not edit!


(when (boundp 'ur3_move::getObjectPosition)
  (if (not (find-package "UR3_MOVE"))
    (make-package "UR3_MOVE"))
  (shadow 'getObjectPosition (find-package "UR3_MOVE")))
(unless (find-package "UR3_MOVE::GETOBJECTPOSITION")
  (make-package "UR3_MOVE::GETOBJECTPOSITION"))
(unless (find-package "UR3_MOVE::GETOBJECTPOSITIONREQUEST")
  (make-package "UR3_MOVE::GETOBJECTPOSITIONREQUEST"))
(unless (find-package "UR3_MOVE::GETOBJECTPOSITIONRESPONSE")
  (make-package "UR3_MOVE::GETOBJECTPOSITIONRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ur3_move::getObjectPositionRequest
  :super ros::object
  :slots (_flag ))

(defmethod ur3_move::getObjectPositionRequest
  (:init
   (&key
    ((:flag __flag) nil)
    )
   (send-super :init)
   (setq _flag __flag)
   self)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:serialization-length
   ()
   (+
    ;; bool _flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _flag
       (if _flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _flag
     (setq _flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur3_move::getObjectPositionResponse
  :super ros::object
  :slots (_target_pose _angle ))

(defmethod ur3_move::getObjectPositionResponse
  (:init
   (&key
    ((:target_pose __target_pose) (instance geometry_msgs::Pose :init))
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _target_pose __target_pose)
   (setq _angle (float __angle))
   self)
  (:target_pose
   (&rest __target_pose)
   (if (keywordp (car __target_pose))
       (send* _target_pose __target_pose)
     (progn
       (if __target_pose (setq _target_pose (car __target_pose)))
       _target_pose)))
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _target_pose
    (send _target_pose :serialization-length)
    ;; float64 _angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _target_pose
       (send _target_pose :serialize s)
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _target_pose
     (send _target_pose :deserialize buf ptr-) (incf ptr- (send _target_pose :serialization-length))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass ur3_move::getObjectPosition
  :super ros::object
  :slots ())

(setf (get ur3_move::getObjectPosition :md5sum-) "914a1e14d98b3c79e57093c7bd6ee205")
(setf (get ur3_move::getObjectPosition :datatype-) "ur3_move/getObjectPosition")
(setf (get ur3_move::getObjectPosition :request) ur3_move::getObjectPositionRequest)
(setf (get ur3_move::getObjectPosition :response) ur3_move::getObjectPositionResponse)

(defmethod ur3_move::getObjectPositionRequest
  (:response () (instance ur3_move::getObjectPositionResponse :init)))

(setf (get ur3_move::getObjectPositionRequest :md5sum-) "914a1e14d98b3c79e57093c7bd6ee205")
(setf (get ur3_move::getObjectPositionRequest :datatype-) "ur3_move/getObjectPositionRequest")
(setf (get ur3_move::getObjectPositionRequest :definition-)
      "bool flag
---
geometry_msgs/Pose target_pose
float64 angle

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get ur3_move::getObjectPositionResponse :md5sum-) "914a1e14d98b3c79e57093c7bd6ee205")
(setf (get ur3_move::getObjectPositionResponse :datatype-) "ur3_move/getObjectPositionResponse")
(setf (get ur3_move::getObjectPositionResponse :definition-)
      "bool flag
---
geometry_msgs/Pose target_pose
float64 angle

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :ur3_move/getObjectPosition "914a1e14d98b3c79e57093c7bd6ee205")


