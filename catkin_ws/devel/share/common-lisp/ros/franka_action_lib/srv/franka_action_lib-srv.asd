
(cl:in-package :asdf)

(defsystem "franka_action_lib-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :franka_action_lib-msg
)
  :components ((:file "_package")
    (:file "GetCurrentRobolibStatusCmd" :depends-on ("_package_GetCurrentRobolibStatusCmd"))
    (:file "_package_GetCurrentRobolibStatusCmd" :depends-on ("_package"))
    (:file "GetCurrentRobotStateCmd" :depends-on ("_package_GetCurrentRobotStateCmd"))
    (:file "_package_GetCurrentRobotStateCmd" :depends-on ("_package"))
  ))