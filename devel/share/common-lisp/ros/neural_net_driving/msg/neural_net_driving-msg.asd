
(cl:in-package :asdf)

(defsystem "neural_net_driving-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "ImageWithID" :depends-on ("_package_ImageWithID"))
    (:file "_package_ImageWithID" :depends-on ("_package"))
  ))