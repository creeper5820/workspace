
(cl:in-package :asdf)

(defsystem "livox_ros_driver2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CustomMsg" :depends-on ("_package_CustomMsg"))
    (:file "_package_CustomMsg" :depends-on ("_package"))
    (:file "CustomPoint" :depends-on ("_package_CustomPoint"))
    (:file "_package_CustomPoint" :depends-on ("_package"))
  ))