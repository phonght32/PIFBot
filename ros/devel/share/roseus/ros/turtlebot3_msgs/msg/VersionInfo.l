;; Auto-generated. Do not edit!


(when (boundp 'turtlebot3_msgs::VersionInfo)
  (if (not (find-package "TURTLEBOT3_MSGS"))
    (make-package "TURTLEBOT3_MSGS"))
  (shadow 'VersionInfo (find-package "TURTLEBOT3_MSGS")))
(unless (find-package "TURTLEBOT3_MSGS::VERSIONINFO")
  (make-package "TURTLEBOT3_MSGS::VERSIONINFO"))

(in-package "ROS")
;;//! \htmlinclude VersionInfo.msg.html


(defclass turtlebot3_msgs::VersionInfo
  :super ros::object
  :slots (_hardware _firmware _software ))

(defmethod turtlebot3_msgs::VersionInfo
  (:init
   (&key
    ((:hardware __hardware) "")
    ((:firmware __firmware) "")
    ((:software __software) "")
    )
   (send-super :init)
   (setq _hardware (string __hardware))
   (setq _firmware (string __firmware))
   (setq _software (string __software))
   self)
  (:hardware
   (&optional __hardware)
   (if __hardware (setq _hardware __hardware)) _hardware)
  (:firmware
   (&optional __firmware)
   (if __firmware (setq _firmware __firmware)) _firmware)
  (:software
   (&optional __software)
   (if __software (setq _software __software)) _software)
  (:serialization-length
   ()
   (+
    ;; string _hardware
    4 (length _hardware)
    ;; string _firmware
    4 (length _firmware)
    ;; string _software
    4 (length _software)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _hardware
       (write-long (length _hardware) s) (princ _hardware s)
     ;; string _firmware
       (write-long (length _firmware) s) (princ _firmware s)
     ;; string _software
       (write-long (length _software) s) (princ _software s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _hardware
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _hardware (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _firmware
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _firmware (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _software
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _software (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get turtlebot3_msgs::VersionInfo :md5sum-) "43e0361461af2970a33107409403ef3c")
(setf (get turtlebot3_msgs::VersionInfo :datatype-) "turtlebot3_msgs/VersionInfo")
(setf (get turtlebot3_msgs::VersionInfo :definition-)
      "########################################
# Messages
########################################
string hardware   # <yyyy>.<mm>.<dd>        : hardware version of Turtlebot3 (ex. 2017.05.23)
string firmware   # <major>.<minor>.<patch> : firmware version of OpenCR
string software   # <major>.<minor>.<patch> : software version of Turtlebot3 ROS packages

")



(provide :turtlebot3_msgs/VersionInfo "43e0361461af2970a33107409403ef3c")


