; Auto-generated. Do not edit!


(cl:in-package turtlebot3_msgs-msg)


;//! \htmlinclude VersionInfo.msg.html

(cl:defclass <VersionInfo> (roslisp-msg-protocol:ros-message)
  ((hardware
    :reader hardware
    :initarg :hardware
    :type cl:string
    :initform "")
   (firmware
    :reader firmware
    :initarg :firmware
    :type cl:string
    :initform "")
   (software
    :reader software
    :initarg :software
    :type cl:string
    :initform ""))
)

(cl:defclass VersionInfo (<VersionInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VersionInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VersionInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name turtlebot3_msgs-msg:<VersionInfo> is deprecated: use turtlebot3_msgs-msg:VersionInfo instead.")))

(cl:ensure-generic-function 'hardware-val :lambda-list '(m))
(cl:defmethod hardware-val ((m <VersionInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtlebot3_msgs-msg:hardware-val is deprecated.  Use turtlebot3_msgs-msg:hardware instead.")
  (hardware m))

(cl:ensure-generic-function 'firmware-val :lambda-list '(m))
(cl:defmethod firmware-val ((m <VersionInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtlebot3_msgs-msg:firmware-val is deprecated.  Use turtlebot3_msgs-msg:firmware instead.")
  (firmware m))

(cl:ensure-generic-function 'software-val :lambda-list '(m))
(cl:defmethod software-val ((m <VersionInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtlebot3_msgs-msg:software-val is deprecated.  Use turtlebot3_msgs-msg:software instead.")
  (software m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VersionInfo>) ostream)
  "Serializes a message object of type '<VersionInfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'hardware))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'hardware))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'firmware))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'firmware))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'software))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'software))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VersionInfo>) istream)
  "Deserializes a message object of type '<VersionInfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hardware) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'hardware) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'firmware) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'firmware) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'software) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'software) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VersionInfo>)))
  "Returns string type for a message object of type '<VersionInfo>"
  "turtlebot3_msgs/VersionInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VersionInfo)))
  "Returns string type for a message object of type 'VersionInfo"
  "turtlebot3_msgs/VersionInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VersionInfo>)))
  "Returns md5sum for a message object of type '<VersionInfo>"
  "43e0361461af2970a33107409403ef3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VersionInfo)))
  "Returns md5sum for a message object of type 'VersionInfo"
  "43e0361461af2970a33107409403ef3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VersionInfo>)))
  "Returns full string definition for message of type '<VersionInfo>"
  (cl:format cl:nil "########################################~%# Messages~%########################################~%string hardware   # <yyyy>.<mm>.<dd>        : hardware version of Turtlebot3 (ex. 2017.05.23)~%string firmware   # <major>.<minor>.<patch> : firmware version of OpenCR~%string software   # <major>.<minor>.<patch> : software version of Turtlebot3 ROS packages~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VersionInfo)))
  "Returns full string definition for message of type 'VersionInfo"
  (cl:format cl:nil "########################################~%# Messages~%########################################~%string hardware   # <yyyy>.<mm>.<dd>        : hardware version of Turtlebot3 (ex. 2017.05.23)~%string firmware   # <major>.<minor>.<patch> : firmware version of OpenCR~%string software   # <major>.<minor>.<patch> : software version of Turtlebot3 ROS packages~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VersionInfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'hardware))
     4 (cl:length (cl:slot-value msg 'firmware))
     4 (cl:length (cl:slot-value msg 'software))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VersionInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'VersionInfo
    (cl:cons ':hardware (hardware msg))
    (cl:cons ':firmware (firmware msg))
    (cl:cons ':software (software msg))
))
