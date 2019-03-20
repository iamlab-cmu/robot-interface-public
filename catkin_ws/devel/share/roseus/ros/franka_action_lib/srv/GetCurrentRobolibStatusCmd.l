;; Auto-generated. Do not edit!


(when (boundp 'franka_action_lib::GetCurrentRobolibStatusCmd)
  (if (not (find-package "FRANKA_ACTION_LIB"))
    (make-package "FRANKA_ACTION_LIB"))
  (shadow 'GetCurrentRobolibStatusCmd (find-package "FRANKA_ACTION_LIB")))
(unless (find-package "FRANKA_ACTION_LIB::GETCURRENTROBOLIBSTATUSCMD")
  (make-package "FRANKA_ACTION_LIB::GETCURRENTROBOLIBSTATUSCMD"))
(unless (find-package "FRANKA_ACTION_LIB::GETCURRENTROBOLIBSTATUSCMDREQUEST")
  (make-package "FRANKA_ACTION_LIB::GETCURRENTROBOLIBSTATUSCMDREQUEST"))
(unless (find-package "FRANKA_ACTION_LIB::GETCURRENTROBOLIBSTATUSCMDRESPONSE")
  (make-package "FRANKA_ACTION_LIB::GETCURRENTROBOLIBSTATUSCMDRESPONSE"))

(in-package "ROS")





(defclass franka_action_lib::GetCurrentRobolibStatusCmdRequest
  :super ros::object
  :slots ())

(defmethod franka_action_lib::GetCurrentRobolibStatusCmdRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass franka_action_lib::GetCurrentRobolibStatusCmdResponse
  :super ros::object
  :slots (_robolib_status ))

(defmethod franka_action_lib::GetCurrentRobolibStatusCmdResponse
  (:init
   (&key
    ((:robolib_status __robolib_status) (instance franka_action_lib::RobolibStatus :init))
    )
   (send-super :init)
   (setq _robolib_status __robolib_status)
   self)
  (:robolib_status
   (&rest __robolib_status)
   (if (keywordp (car __robolib_status))
       (send* _robolib_status __robolib_status)
     (progn
       (if __robolib_status (setq _robolib_status (car __robolib_status)))
       _robolib_status)))
  (:serialization-length
   ()
   (+
    ;; franka_action_lib/RobolibStatus _robolib_status
    (send _robolib_status :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; franka_action_lib/RobolibStatus _robolib_status
       (send _robolib_status :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; franka_action_lib/RobolibStatus _robolib_status
     (send _robolib_status :deserialize buf ptr-) (incf ptr- (send _robolib_status :serialization-length))
   ;;
   self)
  )

(defclass franka_action_lib::GetCurrentRobolibStatusCmd
  :super ros::object
  :slots ())

(setf (get franka_action_lib::GetCurrentRobolibStatusCmd :md5sum-) "4a8606837e9f22dc33518a29b863eeb4")
(setf (get franka_action_lib::GetCurrentRobolibStatusCmd :datatype-) "franka_action_lib/GetCurrentRobolibStatusCmd")
(setf (get franka_action_lib::GetCurrentRobolibStatusCmd :request) franka_action_lib::GetCurrentRobolibStatusCmdRequest)
(setf (get franka_action_lib::GetCurrentRobolibStatusCmd :response) franka_action_lib::GetCurrentRobolibStatusCmdResponse)

(defmethod franka_action_lib::GetCurrentRobolibStatusCmdRequest
  (:response () (instance franka_action_lib::GetCurrentRobolibStatusCmdResponse :init)))

(setf (get franka_action_lib::GetCurrentRobolibStatusCmdRequest :md5sum-) "4a8606837e9f22dc33518a29b863eeb4")
(setf (get franka_action_lib::GetCurrentRobolibStatusCmdRequest :datatype-) "franka_action_lib/GetCurrentRobolibStatusCmdRequest")
(setf (get franka_action_lib::GetCurrentRobolibStatusCmdRequest :definition-)
      "---
RobolibStatus robolib_status

================================================================================
MSG: franka_action_lib/RobolibStatus
# Franka robot state
std_msgs/Header header
bool is_ready
string error_description
bool is_fresh
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
# 0: no frame
# 1: global frame
string frame_id
")

(setf (get franka_action_lib::GetCurrentRobolibStatusCmdResponse :md5sum-) "4a8606837e9f22dc33518a29b863eeb4")
(setf (get franka_action_lib::GetCurrentRobolibStatusCmdResponse :datatype-) "franka_action_lib/GetCurrentRobolibStatusCmdResponse")
(setf (get franka_action_lib::GetCurrentRobolibStatusCmdResponse :definition-)
      "---
RobolibStatus robolib_status

================================================================================
MSG: franka_action_lib/RobolibStatus
# Franka robot state
std_msgs/Header header
bool is_ready
string error_description
bool is_fresh
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
# 0: no frame
# 1: global frame
string frame_id
")



(provide :franka_action_lib/GetCurrentRobolibStatusCmd "4a8606837e9f22dc33518a29b863eeb4")


