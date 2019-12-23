;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::SensorState)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'SensorState (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::SENSORSTATE")
  (make-package "ROBOT_MSGS::SENSORSTATE"))

(in-package "ROS")
;;//! \htmlinclude SensorState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*BUTTON0*" (find-package "ROBOT_MSGS::SENSORSTATE"))
(shadow '*BUTTON0* (find-package "ROBOT_MSGS::SENSORSTATE"))
(defconstant robot_msgs::SensorState::*BUTTON0* 1)
(intern "*BUTTON1*" (find-package "ROBOT_MSGS::SENSORSTATE"))
(shadow '*BUTTON1* (find-package "ROBOT_MSGS::SENSORSTATE"))
(defconstant robot_msgs::SensorState::*BUTTON1* 2)
(intern "*ERROR_LEFT_MOTOR*" (find-package "ROBOT_MSGS::SENSORSTATE"))
(shadow '*ERROR_LEFT_MOTOR* (find-package "ROBOT_MSGS::SENSORSTATE"))
(defconstant robot_msgs::SensorState::*ERROR_LEFT_MOTOR* 1)
(intern "*ERROR_RIGHT_MOTOR*" (find-package "ROBOT_MSGS::SENSORSTATE"))
(shadow '*ERROR_RIGHT_MOTOR* (find-package "ROBOT_MSGS::SENSORSTATE"))
(defconstant robot_msgs::SensorState::*ERROR_RIGHT_MOTOR* 2)
(defclass robot_msgs::SensorState
  :super ros::object
  :slots (_header _led _button _left_encoder _right_encoder _battery ))

(defmethod robot_msgs::SensorState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:led __led) 0)
    ((:button __button) 0)
    ((:left_encoder __left_encoder) 0)
    ((:right_encoder __right_encoder) 0)
    ((:battery __battery) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _led (round __led))
   (setq _button (round __button))
   (setq _left_encoder (round __left_encoder))
   (setq _right_encoder (round __right_encoder))
   (setq _battery (float __battery))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:led
   (&optional __led)
   (if __led (setq _led __led)) _led)
  (:button
   (&optional __button)
   (if __button (setq _button __button)) _button)
  (:left_encoder
   (&optional __left_encoder)
   (if __left_encoder (setq _left_encoder __left_encoder)) _left_encoder)
  (:right_encoder
   (&optional __right_encoder)
   (if __right_encoder (setq _right_encoder __right_encoder)) _right_encoder)
  (:battery
   (&optional __battery)
   (if __battery (setq _battery __battery)) _battery)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _led
    1
    ;; uint8 _button
    1
    ;; int32 _left_encoder
    4
    ;; int32 _right_encoder
    4
    ;; float32 _battery
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _led
       (write-byte _led s)
     ;; uint8 _button
       (write-byte _button s)
     ;; int32 _left_encoder
       (write-long _left_encoder s)
     ;; int32 _right_encoder
       (write-long _right_encoder s)
     ;; float32 _battery
       (sys::poke _battery (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _led
     (setq _led (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _button
     (setq _button (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _left_encoder
     (setq _left_encoder (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _right_encoder
     (setq _right_encoder (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _battery
     (setq _battery (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_msgs::SensorState :md5sum-) "769161449d7d5164923f94701a9ff6e3")
(setf (get robot_msgs::SensorState :datatype-) "robot_msgs/SensorState")
(setf (get robot_msgs::SensorState :definition-)
      "# Button states 
uint8 BUTTON0 = 1
uint8 BUTTON1 = 2

# Motor errors
uint8 ERROR_LEFT_MOTOR  = 1
uint8 ERROR_RIGHT_MOTOR = 2

# Messages
Header  header
uint8   led
uint8  button
int32  left_encoder    
int32  right_encoder   
float32  battery
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :robot_msgs/SensorState "769161449d7d5164923f94701a9ff6e3")


