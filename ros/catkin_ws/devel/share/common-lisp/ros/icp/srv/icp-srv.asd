
(cl:in-package :asdf)

(defsystem "icp-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "ICP" :depends-on ("_package_ICP"))
    (:file "_package_ICP" :depends-on ("_package"))
  ))