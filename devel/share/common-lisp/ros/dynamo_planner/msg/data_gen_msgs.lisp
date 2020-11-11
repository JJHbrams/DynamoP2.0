; Auto-generated. Do not edit!


(cl:in-package dynamo_planner-msg)


;//! \htmlinclude data_gen_msgs.msg.html

(cl:defclass <data_gen_msgs> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil)
   (filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform "")
   (num_data
    :reader num_data
    :initarg :num_data
    :type cl:float
    :initform 0.0))
)

(cl:defclass data_gen_msgs (<data_gen_msgs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <data_gen_msgs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'data_gen_msgs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamo_planner-msg:<data_gen_msgs> is deprecated: use dynamo_planner-msg:data_gen_msgs instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <data_gen_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:flag-val is deprecated.  Use dynamo_planner-msg:flag instead.")
  (flag m))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <data_gen_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:filename-val is deprecated.  Use dynamo_planner-msg:filename instead.")
  (filename m))

(cl:ensure-generic-function 'num_data-val :lambda-list '(m))
(cl:defmethod num_data-val ((m <data_gen_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:num_data-val is deprecated.  Use dynamo_planner-msg:num_data instead.")
  (num_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <data_gen_msgs>) ostream)
  "Serializes a message object of type '<data_gen_msgs>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'num_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <data_gen_msgs>) istream)
  "Deserializes a message object of type '<data_gen_msgs>"
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'num_data) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<data_gen_msgs>)))
  "Returns string type for a message object of type '<data_gen_msgs>"
  "dynamo_planner/data_gen_msgs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'data_gen_msgs)))
  "Returns string type for a message object of type 'data_gen_msgs"
  "dynamo_planner/data_gen_msgs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<data_gen_msgs>)))
  "Returns md5sum for a message object of type '<data_gen_msgs>"
  "5217da41250023fd463ea99124c27fdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'data_gen_msgs)))
  "Returns md5sum for a message object of type 'data_gen_msgs"
  "5217da41250023fd463ea99124c27fdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<data_gen_msgs>)))
  "Returns full string definition for message of type '<data_gen_msgs>"
  (cl:format cl:nil "# bool flag string filename float64 num_data~%bool flag~%string filename~%float64 num_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'data_gen_msgs)))
  "Returns full string definition for message of type 'data_gen_msgs"
  (cl:format cl:nil "# bool flag string filename float64 num_data~%bool flag~%string filename~%float64 num_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <data_gen_msgs>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'filename))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <data_gen_msgs>))
  "Converts a ROS message object to a list"
  (cl:list 'data_gen_msgs
    (cl:cons ':flag (flag msg))
    (cl:cons ':filename (filename msg))
    (cl:cons ':num_data (num_data msg))
))
