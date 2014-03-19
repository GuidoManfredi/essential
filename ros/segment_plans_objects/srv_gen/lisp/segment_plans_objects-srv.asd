
(cl:in-package :asdf)

(defsystem "segment_plans_objects-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :object_recognition_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "PlantopSegmentation" :depends-on ("_package_PlantopSegmentation"))
    (:file "_package_PlantopSegmentation" :depends-on ("_package"))
  ))