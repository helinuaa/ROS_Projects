;; Auto-generated. Do not edit!


(when (boundp 'demo::demo_srv)
  (if (not (find-package "DEMO"))
    (make-package "DEMO"))
  (shadow 'demo_srv (find-package "DEMO")))
(unless (find-package "DEMO::DEMO_SRV")
  (make-package "DEMO::DEMO_SRV"))
(unless (find-package "DEMO::DEMO_SRVREQUEST")
  (make-package "DEMO::DEMO_SRVREQUEST"))
(unless (find-package "DEMO::DEMO_SRVRESPONSE")
  (make-package "DEMO::DEMO_SRVRESPONSE"))

(in-package "ROS")





(defclass demo::demo_srvRequest
  :super ros::object
  :slots (_in ))

(defmethod demo::demo_srvRequest
  (:init
   (&key
    ((:in __in) "")
    )
   (send-super :init)
   (setq _in (string __in))
   self)
  (:in
   (&optional __in)
   (if __in (setq _in __in)) _in)
  (:serialization-length
   ()
   (+
    ;; string _in
    4 (length _in)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _in
       (write-long (length _in) s) (princ _in s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _in
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _in (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass demo::demo_srvResponse
  :super ros::object
  :slots (_out ))

(defmethod demo::demo_srvResponse
  (:init
   (&key
    ((:out __out) "")
    )
   (send-super :init)
   (setq _out (string __out))
   self)
  (:out
   (&optional __out)
   (if __out (setq _out __out)) _out)
  (:serialization-length
   ()
   (+
    ;; string _out
    4 (length _out)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _out
       (write-long (length _out) s) (princ _out s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _out
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _out (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass demo::demo_srv
  :super ros::object
  :slots ())

(setf (get demo::demo_srv :md5sum-) "e21fb7853ad73d6d988d6371d4fed1e2")
(setf (get demo::demo_srv :datatype-) "demo/demo_srv")
(setf (get demo::demo_srv :request) demo::demo_srvRequest)
(setf (get demo::demo_srv :response) demo::demo_srvResponse)

(defmethod demo::demo_srvRequest
  (:response () (instance demo::demo_srvResponse :init)))

(setf (get demo::demo_srvRequest :md5sum-) "e21fb7853ad73d6d988d6371d4fed1e2")
(setf (get demo::demo_srvRequest :datatype-) "demo/demo_srvRequest")
(setf (get demo::demo_srvRequest :definition-)
      "string in

---
string out

")

(setf (get demo::demo_srvResponse :md5sum-) "e21fb7853ad73d6d988d6371d4fed1e2")
(setf (get demo::demo_srvResponse :datatype-) "demo/demo_srvResponse")
(setf (get demo::demo_srvResponse :definition-)
      "string in

---
string out

")



(provide :demo/demo_srv "e21fb7853ad73d6d988d6371d4fed1e2")


