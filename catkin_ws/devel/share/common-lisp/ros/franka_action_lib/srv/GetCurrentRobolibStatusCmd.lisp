; Auto-generated. Do not edit!


(cl:in-package franka_action_lib-srv)


;//! \htmlinclude GetCurrentRobolibStatusCmd-request.msg.html

(cl:defclass <GetCurrentRobolibStatusCmd-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetCurrentRobolibStatusCmd-request (<GetCurrentRobolibStatusCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentRobolibStatusCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentRobolibStatusCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_action_lib-srv:<GetCurrentRobolibStatusCmd-request> is deprecated: use franka_action_lib-srv:GetCurrentRobolibStatusCmd-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentRobolibStatusCmd-request>) ostream)
  "Serializes a message object of type '<GetCurrentRobolibStatusCmd-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentRobolibStatusCmd-request>) istream)
  "Deserializes a message object of type '<GetCurrentRobolibStatusCmd-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentRobolibStatusCmd-request>)))
  "Returns string type for a service object of type '<GetCurrentRobolibStatusCmd-request>"
  "franka_action_lib/GetCurrentRobolibStatusCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentRobolibStatusCmd-request)))
  "Returns string type for a service object of type 'GetCurrentRobolibStatusCmd-request"
  "franka_action_lib/GetCurrentRobolibStatusCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentRobolibStatusCmd-request>)))
  "Returns md5sum for a message object of type '<GetCurrentRobolibStatusCmd-request>"
  "4a8606837e9f22dc33518a29b863eeb4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentRobolibStatusCmd-request)))
  "Returns md5sum for a message object of type 'GetCurrentRobolibStatusCmd-request"
  "4a8606837e9f22dc33518a29b863eeb4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentRobolibStatusCmd-request>)))
  "Returns full string definition for message of type '<GetCurrentRobolibStatusCmd-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentRobolibStatusCmd-request)))
  "Returns full string definition for message of type 'GetCurrentRobolibStatusCmd-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentRobolibStatusCmd-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentRobolibStatusCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentRobolibStatusCmd-request
))
;//! \htmlinclude GetCurrentRobolibStatusCmd-response.msg.html

(cl:defclass <GetCurrentRobolibStatusCmd-response> (roslisp-msg-protocol:ros-message)
  ((robolib_status
    :reader robolib_status
    :initarg :robolib_status
    :type franka_action_lib-msg:RobolibStatus
    :initform (cl:make-instance 'franka_action_lib-msg:RobolibStatus)))
)

(cl:defclass GetCurrentRobolibStatusCmd-response (<GetCurrentRobolibStatusCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentRobolibStatusCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentRobolibStatusCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_action_lib-srv:<GetCurrentRobolibStatusCmd-response> is deprecated: use franka_action_lib-srv:GetCurrentRobolibStatusCmd-response instead.")))

(cl:ensure-generic-function 'robolib_status-val :lambda-list '(m))
(cl:defmethod robolib_status-val ((m <GetCurrentRobolibStatusCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_action_lib-srv:robolib_status-val is deprecated.  Use franka_action_lib-srv:robolib_status instead.")
  (robolib_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentRobolibStatusCmd-response>) ostream)
  "Serializes a message object of type '<GetCurrentRobolibStatusCmd-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robolib_status) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentRobolibStatusCmd-response>) istream)
  "Deserializes a message object of type '<GetCurrentRobolibStatusCmd-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robolib_status) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentRobolibStatusCmd-response>)))
  "Returns string type for a service object of type '<GetCurrentRobolibStatusCmd-response>"
  "franka_action_lib/GetCurrentRobolibStatusCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentRobolibStatusCmd-response)))
  "Returns string type for a service object of type 'GetCurrentRobolibStatusCmd-response"
  "franka_action_lib/GetCurrentRobolibStatusCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentRobolibStatusCmd-response>)))
  "Returns md5sum for a message object of type '<GetCurrentRobolibStatusCmd-response>"
  "4a8606837e9f22dc33518a29b863eeb4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentRobolibStatusCmd-response)))
  "Returns md5sum for a message object of type 'GetCurrentRobolibStatusCmd-response"
  "4a8606837e9f22dc33518a29b863eeb4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentRobolibStatusCmd-response>)))
  "Returns full string definition for message of type '<GetCurrentRobolibStatusCmd-response>"
  (cl:format cl:nil "RobolibStatus robolib_status~%~%================================================================================~%MSG: franka_action_lib/RobolibStatus~%# Franka robot state~%std_msgs/Header header~%bool is_ready~%string error_description~%bool is_fresh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentRobolibStatusCmd-response)))
  "Returns full string definition for message of type 'GetCurrentRobolibStatusCmd-response"
  (cl:format cl:nil "RobolibStatus robolib_status~%~%================================================================================~%MSG: franka_action_lib/RobolibStatus~%# Franka robot state~%std_msgs/Header header~%bool is_ready~%string error_description~%bool is_fresh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentRobolibStatusCmd-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robolib_status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentRobolibStatusCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentRobolibStatusCmd-response
    (cl:cons ':robolib_status (robolib_status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCurrentRobolibStatusCmd)))
  'GetCurrentRobolibStatusCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCurrentRobolibStatusCmd)))
  'GetCurrentRobolibStatusCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentRobolibStatusCmd)))
  "Returns string type for a service object of type '<GetCurrentRobolibStatusCmd>"
  "franka_action_lib/GetCurrentRobolibStatusCmd")