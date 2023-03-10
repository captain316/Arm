;; Auto-generated. Do not edit!


(when (boundp 'gohome::carArm)
  (if (not (find-package "GOHOME"))
    (make-package "GOHOME"))
  (shadow 'carArm (find-package "GOHOME")))
(unless (find-package "GOHOME::CARARM")
  (make-package "GOHOME::CARARM"))
(unless (find-package "GOHOME::CARARMREQUEST")
  (make-package "GOHOME::CARARMREQUEST"))
(unless (find-package "GOHOME::CARARMRESPONSE")
  (make-package "GOHOME::CARARMRESPONSE"))

(in-package "ROS")





(defclass gohome::carArmRequest
  :super ros::object
  :slots (_location ))

(defmethod gohome::carArmRequest
  (:init
   (&key
    ((:location __location) "")
    )
   (send-super :init)
   (setq _location (string __location))
   self)
  (:location
   (&optional __location)
   (if __location (setq _location __location)) _location)
  (:serialization-length
   ()
   (+
    ;; string _location
    4 (length _location)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _location
       (write-long (length _location) s) (princ _location s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _location
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _location (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass gohome::carArmResponse
  :super ros::object
  :slots (_result ))

(defmethod gohome::carArmResponse
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

(defclass gohome::carArm
  :super ros::object
  :slots ())

(setf (get gohome::carArm :md5sum-) "f12396c1cf3ed85338aaa27ae2758520")
(setf (get gohome::carArm :datatype-) "gohome/carArm")
(setf (get gohome::carArm :request) gohome::carArmRequest)
(setf (get gohome::carArm :response) gohome::carArmResponse)

(defmethod gohome::carArmRequest
  (:response () (instance gohome::carArmResponse :init)))

(setf (get gohome::carArmRequest :md5sum-) "f12396c1cf3ed85338aaa27ae2758520")
(setf (get gohome::carArmRequest :datatype-) "gohome/carArmRequest")
(setf (get gohome::carArmRequest :definition-)
      "string location
---
bool result
")

(setf (get gohome::carArmResponse :md5sum-) "f12396c1cf3ed85338aaa27ae2758520")
(setf (get gohome::carArmResponse :datatype-) "gohome/carArmResponse")
(setf (get gohome::carArmResponse :definition-)
      "string location
---
bool result
")



(provide :gohome/carArm "f12396c1cf3ed85338aaa27ae2758520")


