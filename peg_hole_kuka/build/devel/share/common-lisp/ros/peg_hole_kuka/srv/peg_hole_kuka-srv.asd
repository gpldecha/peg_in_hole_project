
(cl:in-package :asdf)

(defsystem "peg_hole_kuka-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "String_cmd" :depends-on ("_package_String_cmd"))
    (:file "_package_String_cmd" :depends-on ("_package"))
  ))