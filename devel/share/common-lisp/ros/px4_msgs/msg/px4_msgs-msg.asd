
(cl:in-package :asdf)

(defsystem "px4_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "VehicleThrustAccSetpoint" :depends-on ("_package_VehicleThrustAccSetpoint"))
    (:file "_package_VehicleThrustAccSetpoint" :depends-on ("_package"))
  ))