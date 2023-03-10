;; Auto-generated. Do not edit!


(when (boundp 'ur3_move::varObjectsPosition)
  (if (not (find-package "UR3_MOVE"))
    (make-package "UR3_MOVE"))
  (shadow 'varObjectsPosition (find-package "UR3_MOVE")))
(unless (find-package "UR3_MOVE::VAROBJECTSPOSITION")
  (make-package "UR3_MOVE::VAROBJECTSPOSITION"))
(unless (find-package "UR3_MOVE::VAROBJECTSPOSITIONREQUEST")
  (make-package "UR3_MOVE::VAROBJECTSPOSITIONREQUEST"))
(unless (find-package "UR3_MOVE::VAROBJECTSPOSITIONRESPONSE")
  (make-package "UR3_MOVE::VAROBJECTSPOSITIONRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ur3_move::varObjectsPositionRequest
  :super ros::object
  :slots (_flag ))

(defmethod ur3_move::varObjectsPositionRequest
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

(defclass ur3_move::varObjectsPositionResponse
  :super ros::object
  :slots (_targets_pose _angles _names ))

(defmethod ur3_move::varObjectsPositionResponse
  (:init
   (&key
    ((:targets_pose __targets_pose) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Pose :init) r)) r))
    ((:angles __angles) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _targets_pose __targets_pose)
   (setq _angles __angles)
   (setq _names __names)
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
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose[] _targets_pose
    (apply #'+ (send-all _targets_pose :serialization-length)) 4
    ;; float64[] _angles
    (* 8    (length _angles)) 4
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
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
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
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
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass ur3_move::varObjectsPosition
  :super ros::object
  :slots ())

(setf (get ur3_move::varObjectsPosition :md5sum-) "471fff188bc2b994ff747dcd01944fb6")
(setf (get ur3_move::varObjectsPosition :datatype-) "ur3_move/varObjectsPosition")
(setf (get ur3_move::varObjectsPosition :request) ur3_move::varObjectsPositionRequest)
(setf (get ur3_move::varObjectsPosition :response) ur3_move::varObjectsPositionResponse)

(defmethod ur3_move::varObjectsPositionRequest
  (:response () (instance ur3_move::varObjectsPositionResponse :init)))

(setf (get ur3_move::varObjectsPositionRequest :md5sum-) "471fff188bc2b994ff747dcd01944fb6")
(setf (get ur3_move::varObjectsPositionRequest :datatype-) "ur3_move/varObjectsPositionRequest")
(setf (get ur3_move::varObjectsPositionRequest :definition-)
      "bool flag
---
geometry_msgs/Pose[] targets_pose
float64[] angles
string[] names

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

(setf (get ur3_move::varObjectsPositionResponse :md5sum-) "471fff188bc2b994ff747dcd01944fb6")
(setf (get ur3_move::varObjectsPositionResponse :datatype-) "ur3_move/varObjectsPositionResponse")
(setf (get ur3_move::varObjectsPositionResponse :definition-)
      "bool flag
---
geometry_msgs/Pose[] targets_pose
float64[] angles
string[] names

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



(provide :ur3_move/varObjectsPosition "471fff188bc2b994ff747dcd01944fb6")


