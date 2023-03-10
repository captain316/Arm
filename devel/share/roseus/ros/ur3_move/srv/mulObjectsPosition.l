;; Auto-generated. Do not edit!


(when (boundp 'ur3_move::mulObjectsPosition)
  (if (not (find-package "UR3_MOVE"))
    (make-package "UR3_MOVE"))
  (shadow 'mulObjectsPosition (find-package "UR3_MOVE")))
(unless (find-package "UR3_MOVE::MULOBJECTSPOSITION")
  (make-package "UR3_MOVE::MULOBJECTSPOSITION"))
(unless (find-package "UR3_MOVE::MULOBJECTSPOSITIONREQUEST")
  (make-package "UR3_MOVE::MULOBJECTSPOSITIONREQUEST"))
(unless (find-package "UR3_MOVE::MULOBJECTSPOSITIONRESPONSE")
  (make-package "UR3_MOVE::MULOBJECTSPOSITIONRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ur3_move::mulObjectsPositionRequest
  :super ros::object
  :slots (_flag ))

(defmethod ur3_move::mulObjectsPositionRequest
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

(defclass ur3_move::mulObjectsPositionResponse
  :super ros::object
  :slots (_targets_pose _angles ))

(defmethod ur3_move::mulObjectsPositionResponse
  (:init
   (&key
    ((:targets_pose __targets_pose) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Pose :init) r)) r))
    ((:angles __angles) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _targets_pose __targets_pose)
   (setq _angles __angles)
   self)
  (:targets_pose
   (&rest __targets_pose)
   (if (keywordp (car __targets_pose))
       (send* _targets_pose __targets_pose)
     (progn
       (if __targets_pose (setq _targets_pose (car __targets_pose)))
       _targets_pose)))
  (:angles
   (&optional __angles)
   (if __angles (setq _angles __angles)) _angles)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose[] _targets_pose
    (apply #'+ (send-all _targets_pose :serialization-length)) 4
    ;; float64[] _angles
    (* 8    (length _angles)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose[] _targets_pose
     (write-long (length _targets_pose) s)
     (dolist (elem _targets_pose)
       (send elem :serialize s)
       )
     ;; float64[] _angles
     (write-long (length _angles) s)
     (dotimes (i (length _angles))
       (sys::poke (elt _angles i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose[] _targets_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _targets_pose (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _targets_pose)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angles (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _angles i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass ur3_move::mulObjectsPosition
  :super ros::object
  :slots ())

(setf (get ur3_move::mulObjectsPosition :md5sum-) "c1c1fd25f8a31070cac107754a1daacf")
(setf (get ur3_move::mulObjectsPosition :datatype-) "ur3_move/mulObjectsPosition")
(setf (get ur3_move::mulObjectsPosition :request) ur3_move::mulObjectsPositionRequest)
(setf (get ur3_move::mulObjectsPosition :response) ur3_move::mulObjectsPositionResponse)

(defmethod ur3_move::mulObjectsPositionRequest
  (:response () (instance ur3_move::mulObjectsPositionResponse :init)))

(setf (get ur3_move::mulObjectsPositionRequest :md5sum-) "c1c1fd25f8a31070cac107754a1daacf")
(setf (get ur3_move::mulObjectsPositionRequest :datatype-) "ur3_move/mulObjectsPositionRequest")
(setf (get ur3_move::mulObjectsPositionRequest :definition-)
      "bool flag
---
geometry_msgs/Pose[] targets_pose
float64[] angles

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

(setf (get ur3_move::mulObjectsPositionResponse :md5sum-) "c1c1fd25f8a31070cac107754a1daacf")
(setf (get ur3_move::mulObjectsPositionResponse :datatype-) "ur3_move/mulObjectsPositionResponse")
(setf (get ur3_move::mulObjectsPositionResponse :definition-)
      "bool flag
---
geometry_msgs/Pose[] targets_pose
float64[] angles

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



(provide :ur3_move/mulObjectsPosition "c1c1fd25f8a31070cac107754a1daacf")


