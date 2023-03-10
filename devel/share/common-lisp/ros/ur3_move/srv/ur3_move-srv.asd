
(cl:in-package :asdf)

(defsystem "ur3_move-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "Stop" :depends-on ("_package_Stop"))
    (:file "_package_Stop" :depends-on ("_package"))
    (:file "getObjectPosition" :depends-on ("_package_getObjectPosition"))
    (:file "_package_getObjectPosition" :depends-on ("_package"))
    (:file "movingObjectPosition" :depends-on ("_package_movingObjectPosition"))
    (:file "_package_movingObjectPosition" :depends-on ("_package"))
    (:file "mulObjectsPosition" :depends-on ("_package_mulObjectsPosition"))
    (:file "_package_mulObjectsPosition" :depends-on ("_package"))
    (:file "varObjectsPosition" :depends-on ("_package_varObjectsPosition"))
    (:file "_package_varObjectsPosition" :depends-on ("_package"))
  ))