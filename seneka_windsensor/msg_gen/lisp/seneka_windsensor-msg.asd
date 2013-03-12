
(cl:in-package :asdf)

(defsystem "seneka_windsensor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "WindData" :depends-on ("_package_WindData"))
    (:file "_package_WindData" :depends-on ("_package"))
  ))