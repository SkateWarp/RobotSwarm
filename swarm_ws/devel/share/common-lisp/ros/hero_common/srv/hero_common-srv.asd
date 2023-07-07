
(cl:in-package :asdf)

(defsystem "hero_common-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetFrequency" :depends-on ("_package_SetFrequency"))
    (:file "_package_SetFrequency" :depends-on ("_package"))
    (:file "SetIRCalibration" :depends-on ("_package_SetIRCalibration"))
    (:file "_package_SetIRCalibration" :depends-on ("_package"))
    (:file "SetMotor" :depends-on ("_package_SetMotor"))
    (:file "_package_SetMotor" :depends-on ("_package"))
    (:file "SetOdom" :depends-on ("_package_SetOdom"))
    (:file "_package_SetOdom" :depends-on ("_package"))
    (:file "SetPID" :depends-on ("_package_SetPID"))
    (:file "_package_SetPID" :depends-on ("_package"))
  ))