;; Auto-generated. Do not edit!


(when (boundp 'ur3_move::movingObjectPosition)
  (if (not (find-package "UR3_MOVE"))
    (make-package "UR3_MOVE"))
  (shadow 'movingObjectPosition (find-package "UR3_MOVE")))
(unless (find-package "UR3_MOVE::MOVINGOBJECTPOSITION")
  (make-package "UR3_MOVE::MOVINGOBJECTPOSITION"))
(unless (find-package "UR3_MOVE::MOVINGOBJECTPOSITIONREQUEST")
  (make-package "UR3_MOVE::MOVINGOBJECTPOSITIONREQUEST"))
(unless (find-package "UR3_MOVE::MOVINGOBJECTPOSITIONRESPONSE")
  (make-package "UR3_MOVE::MOVINGOBJECTPOSITIONRESPONSE"))

(in-package "ROS")





(defclass ur3_move::movingObjectPositionRequest
  :super ros::object
  :slots (_send ))

(defmethod ur3_move::movingObjectPositionRequest
  (:init
   (&key
    ((:send __send) nil)
    )
   (send-super :init)
   (setq _send __send)
   self)
  (:send
   (&optional __send)
   (if __send (setq _send __send)) _send)
  (:serialization-length
   ()
   (+
    ;; bool _send
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _send
       (if _send (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _send
     (setq _send (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur3_move::movingObjectPositionResponse
  :super ros::object
  :slots (_receive ))

(defmethod ur3_move::movingObjectPositionResponse
  (:init
   (&key
    ((:receive __receive) nil)
    )
   (send-super :init)
   (setq _receive __receive)
   self)
  (:receive
   (&optional __receive)
   (if __receive (setq _receive __receive)) _receive)
  (:serialization-length
   ()
   (+
    ;; bool _receive
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _receive
       (if _receive (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _receive
     (setq _receive (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur3_move::movingObjectPosition
  :super ros::object
  :slots ())

(setf (get ur3_move::movingObjectPosition :md5sum-) "05475c87257688fa5ef65958faf092d7")
(setf (get ur3_move::movingObjectPosition :datatype-) "ur3_move/movingObjectPosition")
(setf (get ur3_move::movingObjectPosition :request) ur3_move::movingObjectPositionRequest)
(setf (get ur3_move::movingObjectPosition :response) ur3_move::movingObjectPositionResponse)

(defmethod ur3_move::movingObjectPositionRequest
  (:response () (instance ur3_move::movingObjectPositionResponse :init)))

(setf (get ur3_move::movingObjectPositionRequest :md5sum-) "05475c87257688fa5ef65958faf092d7")
(setf (get ur3_move::movingObjectPositionRequest :datatype-) "ur3_move/movingObjectPositionRequest")
(setf (get ur3_move::movingObjectPositionRequest :definition-)
      "bool send
---
bool receive

")

(setf (get ur3_move::movingObjectPositionResponse :md5sum-) "05475c87257688fa5ef65958faf092d7")
(setf (get ur3_move::movingObjectPositionResponse :datatype-) "ur3_move/movingObjectPositionResponse")
(setf (get ur3_move::movingObjectPositionResponse :definition-)
      "bool send
---
bool receive

")



(provide :ur3_move/movingObjectPosition "05475c87257688fa5ef65958faf092d7")


