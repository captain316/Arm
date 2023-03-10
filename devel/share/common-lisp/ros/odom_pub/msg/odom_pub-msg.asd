
(cl:in-package :asdf)

(defsystem "odom_pub-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "motor_msg" :depends-on ("_package_motor_msg"))
    (:file "_package_motor_msg" :depends-on ("_package"))
  ))