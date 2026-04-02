
(cl:in-package :asdf)

(defsystem "my_class_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MyActionAction" :depends-on ("_package_MyActionAction"))
    (:file "_package_MyActionAction" :depends-on ("_package"))
    (:file "MyActionActionFeedback" :depends-on ("_package_MyActionActionFeedback"))
    (:file "_package_MyActionActionFeedback" :depends-on ("_package"))
    (:file "MyActionActionGoal" :depends-on ("_package_MyActionActionGoal"))
    (:file "_package_MyActionActionGoal" :depends-on ("_package"))
    (:file "MyActionActionResult" :depends-on ("_package_MyActionActionResult"))
    (:file "_package_MyActionActionResult" :depends-on ("_package"))
    (:file "MyActionFeedback" :depends-on ("_package_MyActionFeedback"))
    (:file "_package_MyActionFeedback" :depends-on ("_package"))
    (:file "MyActionGoal" :depends-on ("_package_MyActionGoal"))
    (:file "_package_MyActionGoal" :depends-on ("_package"))
    (:file "MyActionResult" :depends-on ("_package_MyActionResult"))
    (:file "_package_MyActionResult" :depends-on ("_package"))
    (:file "MyMessage" :depends-on ("_package_MyMessage"))
    (:file "_package_MyMessage" :depends-on ("_package"))
  ))