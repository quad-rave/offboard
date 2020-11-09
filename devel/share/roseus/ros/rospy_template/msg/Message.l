;; Auto-generated. Do not edit!


(when (boundp 'rospy_template::Message)
  (if (not (find-package "ROSPY_TEMPLATE"))
    (make-package "ROSPY_TEMPLATE"))
  (shadow 'Message (find-package "ROSPY_TEMPLATE")))
(unless (find-package "ROSPY_TEMPLATE::MESSAGE")
  (make-package "ROSPY_TEMPLATE::MESSAGE"))

(in-package "ROS")
;;//! \htmlinclude Message.msg.html


(defclass rospy_template::Message
  :super ros::object
  :slots (_value ))

(defmethod rospy_template::Message
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

(setf (get rospy_template::Message :md5sum-) "64e58419496c7248b4ef25731f88b8c3")
(setf (get rospy_template::Message :datatype-) "rospy_template/Message")
(setf (get rospy_template::Message :definition-)
      "string value
")



(provide :rospy_template/Message "64e58419496c7248b4ef25731f88b8c3")


