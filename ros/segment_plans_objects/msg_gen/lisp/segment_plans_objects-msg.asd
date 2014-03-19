
(cl:in-package :asdf)

(defsystem "segment_plans_objects-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "PointCloudArray" :depends-on ("_package_PointCloudArray"))
    (:file "_package_PointCloudArray" :depends-on ("_package"))
    (:file "ImageArray" :depends-on ("_package_ImageArray"))
    (:file "_package_ImageArray" :depends-on ("_package"))
  ))