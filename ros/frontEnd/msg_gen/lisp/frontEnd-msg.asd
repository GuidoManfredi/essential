
(cl:in-package :asdf)

(defsystem "frontEnd-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "CloudArray" :depends-on ("_package_CloudArray"))
    (:file "_package_CloudArray" :depends-on ("_package"))
  ))