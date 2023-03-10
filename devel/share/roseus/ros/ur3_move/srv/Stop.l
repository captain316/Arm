;; Auto-generated. Do not edit!


(when (boundp 'ur3_move::Stop)
  (if (not (find-package "UR3_MOVE"))
    (make-package "UR3_MOVE"))
  (shadow 'Stop (find-package "UR3_MOVE")))
(unless (find-package "UR3_MOVE::STOP")
  (make-package "UR3_MOVE::STOP"))
(unless (find-package "UR3_MOVE::STOPREQUEST")
  (make-package "UR3_MOVE::STOPREQUEST"))
(unless (find-package "UR3_MOVE::STOPRESPONSE")
  (make-package "UR3_MOVE::STOPRESPONSE"))

(in-package "ROS")





(defclass ur3_move::StopRequest
  :super ros::object
  :slots (_command ))

(defmethod ur3_move::StopRequest
  (:init
   (&key
    ((:command __command) 0)
    )
   (send-super :init)
   (setq _command (round __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; uint32 _command
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _command
       (write-long _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _command
     (setq _command (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ur3_move::StopResponse
  :super ros::object
  :slots (_result ))

(defmethod ur3_move::StopResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur3_move::Stop
  :super ros::object
  :slots ())

(setf (get ur3_move::Stop :md5sum-) "afc6579bcf2451c763cb3bef58337e2b")
(setf (get ur3_move::Stop :datatype-) "ur3_move/Stop")
(setf (get ur3_move::Stop :request) ur3_move::StopRequest)
(setf (get ur3_move::Stop :response) ur3_move::StopResponse)

(defmethod ur3_move::StopRequest
  (:response () (instance ur3_move::StopResponse :init)))

(setf (get ur3_move::StopRequest :md5sum-) "afc6579bcf2451c763cb3bef58337e2b")
(setf (get ur3_move::StopRequest :datatype-) "ur3_move/StopRequest")
(setf (get ur3_move::StopRequest :definition-)
      "uint32 command

---

bool result
")

(setf (get ur3_move::StopResponse :md5sum-) "afc6579bcf2451c763cb3bef58337e2b")
(setf (get ur3_move::StopResponse :datatype-) "ur3_move/StopResponse")
(setf (get ur3_move::StopResponse :definition-)
      "uint32 command

---

bool result
")



(provide :ur3_move/Stop "afc6579bcf2451c763cb3bef58337e2b")


