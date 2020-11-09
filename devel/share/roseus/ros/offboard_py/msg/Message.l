;; Auto-generated. Do not edit!


(when (boundp 'offboard_py::Message)
  (if (not (find-package "OFFBOARD_PY"))
    (make-package "OFFBOARD_PY"))
  (shadow 'Message (find-package "OFFBOARD_PY")))
(unless (find-package "OFFBOARD_PY::MESSAGE")
  (make-package "OFFBOARD_PY::MESSAGE"))

(in-package "ROS")
;;//! \htmlinclude Message.msg.html


(defclass offboard_py::Message
  :super ros::object
  :slots (_value ))

(defmethod offboard_py::Message
  (:init
   (&key
    ((:value __value) "")
    )
   (send-super :init)
   (setq _value (string __value))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _value
    4 (length _value)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _value
       (write-long (length _value) s) (princ _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _value
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _value (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get offboard_py::Message :md5sum-) "64e58419496c7248b4ef25731f88b8c3")
(setf (get offboard_py::Message :datatype-) "offboard_py/Message")
(setf (get offboard_py::Message :definition-)
      "string value
")



(provide :offboard_py/Message "64e58419496c7248b4ef25731f88b8c3")


