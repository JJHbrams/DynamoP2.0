
(cl:in-package :asdf)

(defsystem "dynamo_planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "physics_data_sampler" :depends-on ("_package_physics_data_sampler"))
    (:file "_package_physics_data_sampler" :depends-on ("_package"))
  ))