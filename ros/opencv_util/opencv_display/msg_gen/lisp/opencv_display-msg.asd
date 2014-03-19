
(cl:in-package :asdf)

(defsystem "opencv_display-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LocaPose" :depends-on ("_package_LocaPose"))
    (:file "_package_LocaPose" :depends-on ("_package"))
  ))