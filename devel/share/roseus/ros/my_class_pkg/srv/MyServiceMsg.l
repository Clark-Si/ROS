;; Auto-generated. Do not edit!


(when (boundp 'my_class_pkg::MyServiceMsg)
  (if (not (find-package "MY_CLASS_PKG"))
    (make-package "MY_CLASS_PKG"))
  (shadow 'MyServiceMsg (find-package "MY_CLASS_PKG")))
(unless (find-package "MY_CLASS_PKG::MYSERVICEMSG")
  (make-package "MY_CLASS_PKG::MYSERVICEMSG"))
(unless (find-package "MY_CLASS_PKG::MYSERVICEMSGREQUEST")
  (make-package "MY_CLASS_PKG::MYSERVICEMSGREQUEST"))
(unless (find-package "MY_CLASS_PKG::MYSERVICEMSGRESPONSE")
  (make-package "MY_CLASS_PKG::MYSERVICEMSGRESPONSE"))

(in-package "ROS")





(defclass my_class_pkg::MyServiceMsgRequest
  :super ros::object
  :slots (_input ))

(defmethod my_class_pkg::MyServiceMsgRequest
  (:init
   (&key
    ((:input __input) 0)
    )
   (send-super :init)
   (setq _input (round __input))
   self)
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:serialization-length
   ()
   (+
    ;; int64 _input
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _input
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _input (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _input) (= (length (_input . bv)) 2)) ;; bignum
              (write-long (ash (elt (_input . bv) 0) 0) s)
              (write-long (ash (elt (_input . bv) 1) -1) s))
             ((and (class _input) (= (length (_input . bv)) 1)) ;; big1
              (write-long (elt (_input . bv) 0) s)
              (write-long (if (>= _input 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _input s)(write-long (if (>= _input 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _input
#+(or :alpha :irix6 :x86_64)
      (setf _input (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _input (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass my_class_pkg::MyServiceMsgResponse
  :super ros::object
  :slots (_output ))

(defmethod my_class_pkg::MyServiceMsgResponse
  (:init
   (&key
    ((:output __output) 0)
    )
   (send-super :init)
   (setq _output (round __output))
   self)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:serialization-length
   ()
   (+
    ;; int64 _output
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _output
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _output (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _output) (= (length (_output . bv)) 2)) ;; bignum
              (write-long (ash (elt (_output . bv) 0) 0) s)
              (write-long (ash (elt (_output . bv) 1) -1) s))
             ((and (class _output) (= (length (_output . bv)) 1)) ;; big1
              (write-long (elt (_output . bv) 0) s)
              (write-long (if (>= _output 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _output s)(write-long (if (>= _output 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _output
#+(or :alpha :irix6 :x86_64)
      (setf _output (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _output (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass my_class_pkg::MyServiceMsg
  :super ros::object
  :slots ())

(setf (get my_class_pkg::MyServiceMsg :md5sum-) "9cc62145209219f185101e72e3ae5c65")
(setf (get my_class_pkg::MyServiceMsg :datatype-) "my_class_pkg/MyServiceMsg")
(setf (get my_class_pkg::MyServiceMsg :request) my_class_pkg::MyServiceMsgRequest)
(setf (get my_class_pkg::MyServiceMsg :response) my_class_pkg::MyServiceMsgResponse)

(defmethod my_class_pkg::MyServiceMsgRequest
  (:response () (instance my_class_pkg::MyServiceMsgResponse :init)))

(setf (get my_class_pkg::MyServiceMsgRequest :md5sum-) "9cc62145209219f185101e72e3ae5c65")
(setf (get my_class_pkg::MyServiceMsgRequest :datatype-) "my_class_pkg/MyServiceMsgRequest")
(setf (get my_class_pkg::MyServiceMsgRequest :definition-)
      "int64 input # 查询
---
int64 output # 回复

")

(setf (get my_class_pkg::MyServiceMsgResponse :md5sum-) "9cc62145209219f185101e72e3ae5c65")
(setf (get my_class_pkg::MyServiceMsgResponse :datatype-) "my_class_pkg/MyServiceMsgResponse")
(setf (get my_class_pkg::MyServiceMsgResponse :definition-)
      "int64 input # 查询
---
int64 output # 回复

")



(provide :my_class_pkg/MyServiceMsg "9cc62145209219f185101e72e3ae5c65")


