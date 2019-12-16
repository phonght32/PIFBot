;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::Sound)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'Sound (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::SOUND")
  (make-package "ROBOT_MSGS::SOUND"))

(in-package "ROS")
;;//! \htmlinclude Sound.msg.html


(intern "*OFF*" (find-package "ROBOT_MSGS::SOUND"))
(shadow '*OFF* (find-package "ROBOT_MSGS::SOUND"))
(defconstant robot_msgs::Sound::*OFF* 0)
(intern "*ON*" (find-package "ROBOT_MSGS::SOUND"))
(shadow '*ON* (find-package "ROBOT_MSGS::SOUND"))
(defconstant robot_msgs::Sound::*ON* 1)
(intern "*LOW_BATTERY*" (find-package "ROBOT_MSGS::SOUND"))
(shadow '*LOW_BATTERY* (find-package "ROBOT_MSGS::SOUND"))
(defconstant robot_msgs::Sound::*LOW_BATTERY* 2)
(intern "*ERROR*" (find-package "ROBOT_MSGS::SOUND"))
(shadow '*ERROR* (find-package "ROBOT_MSGS::SOUND"))
(defconstant robot_msgs::Sound::*ERROR* 3)
(intern "*BUTTON1*" (find-package "ROBOT_MSGS::SOUND"))
(shadow '*BUTTON1* (find-package "ROBOT_MSGS::SOUND"))
(defconstant robot_msgs::Sound::*BUTTON1* 4)
(intern "*BUTTON2*" (find-package "ROBOT_MSGS::SOUND"))
(shadow '*BUTTON2* (find-package "ROBOT_MSGS::SOUND"))
(defconstant robot_msgs::Sound::*BUTTON2* 5)
(defclass robot_msgs::Sound
  :super ros::object
  :slots (_value ))

(defmethod robot_msgs::Sound
  (:init
   (&key
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _value (round __value))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; uint8 _value
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _value
       (write-byte _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _value
     (setq _value (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get robot_msgs::Sound :md5sum-) "e1f8c7f8a9a61383b5734fbdeca2f99a")
(setf (get robot_msgs::Sound :datatype-) "robot_msgs/Sound")
(setf (get robot_msgs::Sound :definition-)
      "########################################
# CONSTANTS
########################################
uint8 OFF           = 0
uint8 ON            = 1
uint8 LOW_BATTERY   = 2
uint8 ERROR         = 3
uint8 BUTTON1       = 4
uint8 BUTTON2       = 5

########################################
# Messages
########################################
uint8 value
")



(provide :robot_msgs/Sound "e1f8c7f8a9a61383b5734fbdeca2f99a")


