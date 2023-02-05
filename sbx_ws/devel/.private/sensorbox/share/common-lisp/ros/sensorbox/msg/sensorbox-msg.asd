
(cl:in-package :asdf)

(defsystem "sensorbox-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AQI" :depends-on ("_package_AQI"))
    (:file "_package_AQI" :depends-on ("_package"))
  ))