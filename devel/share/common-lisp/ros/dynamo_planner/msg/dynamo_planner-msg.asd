
(cl:in-package :asdf)

(defsystem "dynamo_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "custom_states_msgs" :depends-on ("_package_custom_states_msgs"))
    (:file "_package_custom_states_msgs" :depends-on ("_package"))
  ))