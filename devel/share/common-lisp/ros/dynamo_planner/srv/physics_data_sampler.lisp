; Auto-generated. Do not edit!


(cl:in-package dynamo_planner-srv)


;//! \htmlinclude physics_data_sampler-request.msg.html

(cl:defclass <physics_data_sampler-request> (roslisp-msg-protocol:ros-message)
  ((NUM_STEP
    :reader NUM_STEP
    :initarg :NUM_STEP
    :type cl:integer
    :initform 0))
)

(cl:defclass physics_data_sampler-request (<physics_data_sampler-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <physics_data_sampler-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'physics_data_sampler-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamo_planner-srv:<physics_data_sampler-request> is deprecated: use dynamo_planner-srv:physics_data_sampler-request instead.")))

(cl:ensure-generic-function 'NUM_STEP-val :lambda-list '(m))
(cl:defmethod NUM_STEP-val ((m <physics_data_sampler-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-srv:NUM_STEP-val is deprecated.  Use dynamo_planner-srv:NUM_STEP instead.")
  (NUM_STEP m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <physics_data_sampler-request>) ostream)
  "Serializes a message object of type '<physics_data_sampler-request>"
  (cl:let* ((signed (cl:slot-value msg 'NUM_STEP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <physics_data_sampler-request>) istream)
  "Deserializes a message object of type '<physics_data_sampler-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'NUM_STEP) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<physics_data_sampler-request>)))
  "Returns string type for a service object of type '<physics_data_sampler-request>"
  "dynamo_planner/physics_data_samplerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'physics_data_sampler-request)))
  "Returns string type for a service object of type 'physics_data_sampler-request"
  "dynamo_planner/physics_data_samplerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<physics_data_sampler-request>)))
  "Returns md5sum for a message object of type '<physics_data_sampler-request>"
  "d13b09ac032499e03ad2334da1a598d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'physics_data_sampler-request)))
  "Returns md5sum for a message object of type 'physics_data_sampler-request"
  "d13b09ac032499e03ad2334da1a598d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<physics_data_sampler-request>)))
  "Returns full string definition for message of type '<physics_data_sampler-request>"
  (cl:format cl:nil "~%int64 NUM_STEP~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'physics_data_sampler-request)))
  "Returns full string definition for message of type 'physics_data_sampler-request"
  (cl:format cl:nil "~%int64 NUM_STEP~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <physics_data_sampler-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <physics_data_sampler-request>))
  "Converts a ROS message object to a list"
  (cl:list 'physics_data_sampler-request
    (cl:cons ':NUM_STEP (NUM_STEP msg))
))
;//! \htmlinclude physics_data_sampler-response.msg.html

(cl:defclass <physics_data_sampler-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass physics_data_sampler-response (<physics_data_sampler-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <physics_data_sampler-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'physics_data_sampler-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamo_planner-srv:<physics_data_sampler-response> is deprecated: use dynamo_planner-srv:physics_data_sampler-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <physics_data_sampler-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamo_planner-srv:result-val is deprecated.  Use dynamo_planner-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <physics_data_sampler-response>) ostream)
  "Serializes a message object of type '<physics_data_sampler-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <physics_data_sampler-response>) istream)
  "Deserializes a message object of type '<physics_data_sampler-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<physics_data_sampler-response>)))
  "Returns string type for a service object of type '<physics_data_sampler-response>"
  "dynamo_planner/physics_data_samplerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'physics_data_sampler-response)))
  "Returns string type for a service object of type 'physics_data_sampler-response"
  "dynamo_planner/physics_data_samplerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<physics_data_sampler-response>)))
  "Returns md5sum for a message object of type '<physics_data_sampler-response>"
  "d13b09ac032499e03ad2334da1a598d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'physics_data_sampler-response)))
  "Returns md5sum for a message object of type 'physics_data_sampler-response"
  "d13b09ac032499e03ad2334da1a598d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<physics_data_sampler-response>)))
  "Returns full string definition for message of type '<physics_data_sampler-response>"
  (cl:format cl:nil "~%bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'physics_data_sampler-response)))
  "Returns full string definition for message of type 'physics_data_sampler-response"
  (cl:format cl:nil "~%bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <physics_data_sampler-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <physics_data_sampler-response>))
  "Converts a ROS message object to a list"
  (cl:list 'physics_data_sampler-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'physics_data_sampler)))
  'physics_data_sampler-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'physics_data_sampler)))
  'physics_data_sampler-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'physics_data_sampler)))
  "Returns string type for a service object of type '<physics_data_sampler>"
  "dynamo_planner/physics_data_sampler")