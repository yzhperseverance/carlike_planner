
(cl:in-package :asdf)

(defsystem "mpc_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "SE2Traj" :depends-on ("_package_SE2Traj"))
    (:file "_package_SE2Traj" :depends-on ("_package"))
  ))