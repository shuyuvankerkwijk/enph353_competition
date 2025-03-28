
(cl:in-package :asdf)

(defsystem "neural_net_driving-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "ImageProcessor" :depends-on ("_package_ImageProcessor"))
    (:file "_package_ImageProcessor" :depends-on ("_package"))
  ))