
(cl:in-package :asdf)

(defsystem "gohome-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "carArm" :depends-on ("_package_carArm"))
    (:file "_package_carArm" :depends-on ("_package"))
  ))