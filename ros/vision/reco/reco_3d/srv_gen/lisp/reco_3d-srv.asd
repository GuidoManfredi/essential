
(cl:in-package :asdf)

(defsystem "reco_3d-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "OrientedBoundingBoxRecognition" :depends-on ("_package_OrientedBoundingBoxRecognition"))
    (:file "_package_OrientedBoundingBoxRecognition" :depends-on ("_package"))
  ))