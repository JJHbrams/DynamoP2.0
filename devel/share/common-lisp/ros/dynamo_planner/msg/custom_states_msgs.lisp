; Auto-generated. Do not edit!


(cl:in-package dynamo_planner-msg)


;//! \htmlinclude custom_states_msgs.msg.html

(cl:defclass <custom_states_msgs> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (controlX
    :reader controlX
    :initarg :controlX
    :type cl:float
    :initform 0.0)
   (controlY
    :reader controlY
    :initarg :controlY
    :type cl:float
    :initform 0.0)
   (controlYAW
    :reader controlYAW
    :initarg :controlYAW
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0)
   (pre_x
    :reader pre_x
    :initarg :pre_x
    :type cl:float
    :initform 0.0)
   (pre_y
    :reader pre_y
    :initarg :pre_y
    :type cl:float
    :initform 0.0)
   (pre_yaw
    :reader pre_yaw
    :initarg :pre_yaw
    :type cl:float
    :initform 0.0)
   (flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil)
   (tf_flag
    :reader tf_flag
    :initarg :tf_flag
    :type cl:boolean
    :initform cl:nil)
   (diff_x
    :reader diff_x
    :initarg :diff_x
    :type cl:boolean
    :initform cl:nil)
   (diff_y
    :reader diff_y
    :initarg :diff_y
    :type cl:boolean
    :initform cl:nil)
   (diff_yaw
    :reader diff_yaw
    :initarg :diff_yaw
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass custom_states_msgs (<custom_states_msgs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <custom_states_msgs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'custom_states_msgs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamo_planner-msg:<custom_states_msgs> is deprecated: use dynamo_planner-msg:custom_states_msgs instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:x-val is deprecated.  Use dynamo_planner-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:y-val is deprecated.  Use dynamo_planner-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:yaw-val is deprecated.  Use dynamo_planner-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'controlX-val :lambda-list '(m))
(cl:defmethod controlX-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:controlX-val is deprecated.  Use dynamo_planner-msg:controlX instead.")
  (controlX m))

(cl:ensure-generic-function 'controlY-val :lambda-list '(m))
(cl:defmethod controlY-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:controlY-val is deprecated.  Use dynamo_planner-msg:controlY instead.")
  (controlY m))

(cl:ensure-generic-function 'controlYAW-val :lambda-list '(m))
(cl:defmethod controlYAW-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:controlYAW-val is deprecated.  Use dynamo_planner-msg:controlYAW instead.")
  (controlYAW m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:duration-val is deprecated.  Use dynamo_planner-msg:duration instead.")
  (duration m))

(cl:ensure-generic-function 'pre_x-val :lambda-list '(m))
(cl:defmethod pre_x-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:pre_x-val is deprecated.  Use dynamo_planner-msg:pre_x instead.")
  (pre_x m))

(cl:ensure-generic-function 'pre_y-val :lambda-list '(m))
(cl:defmethod pre_y-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:pre_y-val is deprecated.  Use dynamo_planner-msg:pre_y instead.")
  (pre_y m))

(cl:ensure-generic-function 'pre_yaw-val :lambda-list '(m))
(cl:defmethod pre_yaw-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:pre_yaw-val is deprecated.  Use dynamo_planner-msg:pre_yaw instead.")
  (pre_yaw m))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:flag-val is deprecated.  Use dynamo_planner-msg:flag instead.")
  (flag m))

(cl:ensure-generic-function 'tf_flag-val :lambda-list '(m))
(cl:defmethod tf_flag-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:tf_flag-val is deprecated.  Use dynamo_planner-msg:tf_flag instead.")
  (tf_flag m))

(cl:ensure-generic-function 'diff_x-val :lambda-list '(m))
(cl:defmethod diff_x-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:diff_x-val is deprecated.  Use dynamo_planner-msg:diff_x instead.")
  (diff_x m))

(cl:ensure-generic-function 'diff_y-val :lambda-list '(m))
(cl:defmethod diff_y-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:diff_y-val is deprecated.  Use dynamo_planner-msg:diff_y instead.")
  (diff_y m))

(cl:ensure-generic-function 'diff_yaw-val :lambda-list '(m))
(cl:defmethod diff_yaw-val ((m <custom_states_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-msg:diff_yaw-val is deprecated.  Use dynamo_planner-msg:diff_yaw instead.")
  (diff_yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <custom_states_msgs>) ostream)
  "Serializes a message object of type '<custom_states_msgs>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'controlX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'controlY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'controlYAW))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pre_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pre_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pre_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tf_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'diff_x) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'diff_y) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'diff_yaw) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <custom_states_msgs>) istream)
  "Deserializes a message object of type '<custom_states_msgs>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'controlX) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'controlY) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'controlYAW) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pre_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pre_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pre_yaw) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'tf_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'diff_x) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'diff_y) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'diff_yaw) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<custom_states_msgs>)))
  "Returns string type for a message object of type '<custom_states_msgs>"
  "dynamo_planner/custom_states_msgs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'custom_states_msgs)))
  "Returns string type for a message object of type 'custom_states_msgs"
  "dynamo_planner/custom_states_msgs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<custom_states_msgs>)))
  "Returns md5sum for a message object of type '<custom_states_msgs>"
  "3b52bf44a08210bd7d5f8e100b5f756a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'custom_states_msgs)))
  "Returns md5sum for a message object of type 'custom_states_msgs"
  "3b52bf44a08210bd7d5f8e100b5f756a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<custom_states_msgs>)))
  "Returns full string definition for message of type '<custom_states_msgs>"
  (cl:format cl:nil "# double type x y yaw control duration~%float64 x~%float64 y~%float64 yaw~%~%float64 controlX~%float64 controlY~%float64 controlYAW~%~%float64 duration~%~%float64 pre_x~%float64 pre_y~%float64 pre_yaw~%~%bool flag~%bool tf_flag~%~%bool diff_x~%bool diff_y~%bool diff_yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'custom_states_msgs)))
  "Returns full string definition for message of type 'custom_states_msgs"
  (cl:format cl:nil "# double type x y yaw control duration~%float64 x~%float64 y~%float64 yaw~%~%float64 controlX~%float64 controlY~%float64 controlYAW~%~%float64 duration~%~%float64 pre_x~%float64 pre_y~%float64 pre_yaw~%~%bool flag~%bool tf_flag~%~%bool diff_x~%bool diff_y~%bool diff_yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <custom_states_msgs>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <custom_states_msgs>))
  "Converts a ROS message object to a list"
  (cl:list 'custom_states_msgs
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':controlX (controlX msg))
    (cl:cons ':controlY (controlY msg))
    (cl:cons ':controlYAW (controlYAW msg))
    (cl:cons ':duration (duration msg))
    (cl:cons ':pre_x (pre_x msg))
    (cl:cons ':pre_y (pre_y msg))
    (cl:cons ':pre_yaw (pre_yaw msg))
    (cl:cons ':flag (flag msg))
    (cl:cons ':tf_flag (tf_flag msg))
    (cl:cons ':diff_x (diff_x msg))
    (cl:cons ':diff_y (diff_y msg))
    (cl:cons ':diff_yaw (diff_yaw msg))
))
