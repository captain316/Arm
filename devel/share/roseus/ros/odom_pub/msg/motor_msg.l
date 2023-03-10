;; Auto-generated. Do not edit!


(when (boundp 'odom_pub::motor_msg)
  (if (not (find-package "ODOM_PUB"))
    (make-package "ODOM_PUB"))
  (shadow 'motor_msg (find-package "ODOM_PUB")))
(unless (find-package "ODOM_PUB::MOTOR_MSG")
  (make-package "ODOM_PUB::MOTOR_MSG"))

(in-package "ROS")
;;//! \htmlinclude motor_msg.msg.html


(defclass odom_pub::motor_msg
  :super ros::object
  :slots (_driver_motor_01 _driver_motor_02 _driver_motor_03 _driver_motor_04 _steer_motor_01 _steer_motor_02 _steer_motor_03 _steer_motor_04 ))

(defmethod odom_pub::motor_msg
  (:init
   (&key
    ((:driver_motor_01 __driver_motor_01) 0.0)
    ((:driver_motor_02 __driver_motor_02) 0.0)
    ((:driver_motor_03 __driver_motor_03) 0.0)
    ((:driver_motor_04 __driver_motor_04) 0.0)
    ((:steer_motor_01 __steer_motor_01) 0.0)
    ((:steer_motor_02 __steer_motor_02) 0.0)
    ((:steer_motor_03 __steer_motor_03) 0.0)
    ((:steer_motor_04 __steer_motor_04) 0.0)
    )
   (send-super :init)
   (setq _driver_motor_01 (float __driver_motor_01))
   (setq _driver_motor_02 (float __driver_motor_02))
   (setq _driver_motor_03 (float __driver_motor_03))
   (setq _driver_motor_04 (float __driver_motor_04))
   (setq _steer_motor_01 (float __steer_motor_01))
   (setq _steer_motor_02 (float __steer_motor_02))
   (setq _steer_motor_03 (float __steer_motor_03))
   (setq _steer_motor_04 (float __steer_motor_04))
   self)
  (:driver_motor_01
   (&optional __driver_motor_01)
   (if __driver_motor_01 (setq _driver_motor_01 __driver_motor_01)) _driver_motor_01)
  (:driver_motor_02
   (&optional __driver_motor_02)
   (if __driver_motor_02 (setq _driver_motor_02 __driver_motor_02)) _driver_motor_02)
  (:driver_motor_03
   (&optional __driver_motor_03)
   (if __driver_motor_03 (setq _driver_motor_03 __driver_motor_03)) _driver_motor_03)
  (:driver_motor_04
   (&optional __driver_motor_04)
   (if __driver_motor_04 (setq _driver_motor_04 __driver_motor_04)) _driver_motor_04)
  (:steer_motor_01
   (&optional __steer_motor_01)
   (if __steer_motor_01 (setq _steer_motor_01 __steer_motor_01)) _steer_motor_01)
  (:steer_motor_02
   (&optional __steer_motor_02)
   (if __steer_motor_02 (setq _steer_motor_02 __steer_motor_02)) _steer_motor_02)
  (:steer_motor_03
   (&optional __steer_motor_03)
   (if __steer_motor_03 (setq _steer_motor_03 __steer_motor_03)) _steer_motor_03)
  (:steer_motor_04
   (&optional __steer_motor_04)
   (if __steer_motor_04 (setq _steer_motor_04 __steer_motor_04)) _steer_motor_04)
  (:serialization-length
   ()
   (+
    ;; float64 _driver_motor_01
    8
    ;; float64 _driver_motor_02
    8
    ;; float64 _driver_motor_03
    8
    ;; float64 _driver_motor_04
    8
    ;; float64 _steer_motor_01
    8
    ;; float64 _steer_motor_02
    8
    ;; float64 _steer_motor_03
    8
    ;; float64 _steer_motor_04
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _driver_motor_01
       (sys::poke _driver_motor_01 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _driver_motor_02
       (sys::poke _driver_motor_02 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _driver_motor_03
       (sys::poke _driver_motor_03 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _driver_motor_04
       (sys::poke _driver_motor_04 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steer_motor_01
       (sys::poke _steer_motor_01 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steer_motor_02
       (sys::poke _steer_motor_02 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steer_motor_03
       (sys::poke _steer_motor_03 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steer_motor_04
       (sys::poke _steer_motor_04 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _driver_motor_01
     (setq _driver_motor_01 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _driver_motor_02
     (setq _driver_motor_02 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _driver_motor_03
     (setq _driver_motor_03 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _driver_motor_04
     (setq _driver_motor_04 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steer_motor_01
     (setq _steer_motor_01 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steer_motor_02
     (setq _steer_motor_02 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steer_motor_03
     (setq _steer_motor_03 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steer_motor_04
     (setq _steer_motor_04 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get odom_pub::motor_msg :md5sum-) "0e4c0fdbf8c8204c136aa0e02d2bf289")
(setf (get odom_pub::motor_msg :datatype-) "odom_pub/motor_msg")
(setf (get odom_pub::motor_msg :definition-)
      "float64 driver_motor_01     
float64 driver_motor_02
float64 driver_motor_03
float64 driver_motor_04

float64 steer_motor_01
float64 steer_motor_02
float64 steer_motor_03
float64 steer_motor_04


")



(provide :odom_pub/motor_msg "0e4c0fdbf8c8204c136aa0e02d2bf289")


