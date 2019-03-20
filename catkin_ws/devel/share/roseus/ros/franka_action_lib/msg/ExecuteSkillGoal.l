;; Auto-generated. Do not edit!


(when (boundp 'franka_action_lib::ExecuteSkillGoal)
  (if (not (find-package "FRANKA_ACTION_LIB"))
    (make-package "FRANKA_ACTION_LIB"))
  (shadow 'ExecuteSkillGoal (find-package "FRANKA_ACTION_LIB")))
(unless (find-package "FRANKA_ACTION_LIB::EXECUTESKILLGOAL")
  (make-package "FRANKA_ACTION_LIB::EXECUTESKILLGOAL"))

(in-package "ROS")
;;//! \htmlinclude ExecuteSkillGoal.msg.html


(defclass franka_action_lib::ExecuteSkillGoal
  :super ros::object
  :slots (_skill_type _skill_description _meta_skill_type _meta_skill_id _sensor_topics _sensor_value_sizes _initial_sensor_values _traj_gen_type _num_traj_gen_params _traj_gen_params _feedback_controller_type _num_feedback_controller_params _feedback_controller_params _termination_type _num_termination_params _termination_params _timer_type _num_timer_params _timer_params ))

(defmethod franka_action_lib::ExecuteSkillGoal
  (:init
   (&key
    ((:skill_type __skill_type) 0)
    ((:skill_description __skill_description) "")
    ((:meta_skill_type __meta_skill_type) 0)
    ((:meta_skill_id __meta_skill_id) 0)
    ((:sensor_topics __sensor_topics) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:sensor_value_sizes __sensor_value_sizes) (make-array 0 :initial-element 0 :element-type :integer))
    ((:initial_sensor_values __initial_sensor_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:traj_gen_type __traj_gen_type) 0)
    ((:num_traj_gen_params __num_traj_gen_params) 0)
    ((:traj_gen_params __traj_gen_params) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:feedback_controller_type __feedback_controller_type) 0)
    ((:num_feedback_controller_params __num_feedback_controller_params) 0)
    ((:feedback_controller_params __feedback_controller_params) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:termination_type __termination_type) 0)
    ((:num_termination_params __num_termination_params) 0)
    ((:termination_params __termination_params) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:timer_type __timer_type) 0)
    ((:num_timer_params __num_timer_params) 0)
    ((:timer_params __timer_params) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _skill_type (round __skill_type))
   (setq _skill_description (string __skill_description))
   (setq _meta_skill_type (round __meta_skill_type))
   (setq _meta_skill_id (round __meta_skill_id))
   (setq _sensor_topics __sensor_topics)
   (setq _sensor_value_sizes __sensor_value_sizes)
   (setq _initial_sensor_values __initial_sensor_values)
   (setq _traj_gen_type (round __traj_gen_type))
   (setq _num_traj_gen_params (round __num_traj_gen_params))
   (setq _traj_gen_params __traj_gen_params)
   (setq _feedback_controller_type (round __feedback_controller_type))
   (setq _num_feedback_controller_params (round __num_feedback_controller_params))
   (setq _feedback_controller_params __feedback_controller_params)
   (setq _termination_type (round __termination_type))
   (setq _num_termination_params (round __num_termination_params))
   (setq _termination_params __termination_params)
   (setq _timer_type (round __timer_type))
   (setq _num_timer_params (round __num_timer_params))
   (setq _timer_params __timer_params)
   self)
  (:skill_type
   (&optional __skill_type)
   (if __skill_type (setq _skill_type __skill_type)) _skill_type)
  (:skill_description
   (&optional __skill_description)
   (if __skill_description (setq _skill_description __skill_description)) _skill_description)
  (:meta_skill_type
   (&optional __meta_skill_type)
   (if __meta_skill_type (setq _meta_skill_type __meta_skill_type)) _meta_skill_type)
  (:meta_skill_id
   (&optional __meta_skill_id)
   (if __meta_skill_id (setq _meta_skill_id __meta_skill_id)) _meta_skill_id)
  (:sensor_topics
   (&optional __sensor_topics)
   (if __sensor_topics (setq _sensor_topics __sensor_topics)) _sensor_topics)
  (:sensor_value_sizes
   (&optional __sensor_value_sizes)
   (if __sensor_value_sizes (setq _sensor_value_sizes __sensor_value_sizes)) _sensor_value_sizes)
  (:initial_sensor_values
   (&optional __initial_sensor_values)
   (if __initial_sensor_values (setq _initial_sensor_values __initial_sensor_values)) _initial_sensor_values)
  (:traj_gen_type
   (&optional __traj_gen_type)
   (if __traj_gen_type (setq _traj_gen_type __traj_gen_type)) _traj_gen_type)
  (:num_traj_gen_params
   (&optional __num_traj_gen_params)
   (if __num_traj_gen_params (setq _num_traj_gen_params __num_traj_gen_params)) _num_traj_gen_params)
  (:traj_gen_params
   (&optional __traj_gen_params)
   (if __traj_gen_params (setq _traj_gen_params __traj_gen_params)) _traj_gen_params)
  (:feedback_controller_type
   (&optional __feedback_controller_type)
   (if __feedback_controller_type (setq _feedback_controller_type __feedback_controller_type)) _feedback_controller_type)
  (:num_feedback_controller_params
   (&optional __num_feedback_controller_params)
   (if __num_feedback_controller_params (setq _num_feedback_controller_params __num_feedback_controller_params)) _num_feedback_controller_params)
  (:feedback_controller_params
   (&optional __feedback_controller_params)
   (if __feedback_controller_params (setq _feedback_controller_params __feedback_controller_params)) _feedback_controller_params)
  (:termination_type
   (&optional __termination_type)
   (if __termination_type (setq _termination_type __termination_type)) _termination_type)
  (:num_termination_params
   (&optional __num_termination_params)
   (if __num_termination_params (setq _num_termination_params __num_termination_params)) _num_termination_params)
  (:termination_params
   (&optional __termination_params)
   (if __termination_params (setq _termination_params __termination_params)) _termination_params)
  (:timer_type
   (&optional __timer_type)
   (if __timer_type (setq _timer_type __timer_type)) _timer_type)
  (:num_timer_params
   (&optional __num_timer_params)
   (if __num_timer_params (setq _num_timer_params __num_timer_params)) _num_timer_params)
  (:timer_params
   (&optional __timer_params)
   (if __timer_params (setq _timer_params __timer_params)) _timer_params)
  (:serialization-length
   ()
   (+
    ;; int64 _skill_type
    8
    ;; string _skill_description
    4 (length _skill_description)
    ;; int64 _meta_skill_type
    8
    ;; int64 _meta_skill_id
    8
    ;; string[] _sensor_topics
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _sensor_topics)) 4
    ;; uint64[] _sensor_value_sizes
    (* 8    (length _sensor_value_sizes)) 4
    ;; float64[] _initial_sensor_values
    (* 8    (length _initial_sensor_values)) 4
    ;; int64 _traj_gen_type
    8
    ;; uint64 _num_traj_gen_params
    8
    ;; float64[] _traj_gen_params
    (* 8    (length _traj_gen_params)) 4
    ;; int64 _feedback_controller_type
    8
    ;; uint64 _num_feedback_controller_params
    8
    ;; float64[] _feedback_controller_params
    (* 8    (length _feedback_controller_params)) 4
    ;; int64 _termination_type
    8
    ;; uint64 _num_termination_params
    8
    ;; float64[] _termination_params
    (* 8    (length _termination_params)) 4
    ;; int64 _timer_type
    8
    ;; uint64 _num_timer_params
    8
    ;; float64[] _timer_params
    (* 8    (length _timer_params)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _skill_type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _skill_type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _skill_type) (= (length (_skill_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_skill_type . bv) 0) 0) s)
              (write-long (ash (elt (_skill_type . bv) 1) -1) s))
             ((and (class _skill_type) (= (length (_skill_type . bv)) 1)) ;; big1
              (write-long (elt (_skill_type . bv) 0) s)
              (write-long (if (>= _skill_type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _skill_type s)(write-long (if (>= _skill_type 0) 0 #xffffffff) s)))
     ;; string _skill_description
       (write-long (length _skill_description) s) (princ _skill_description s)
     ;; int64 _meta_skill_type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _meta_skill_type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _meta_skill_type) (= (length (_meta_skill_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_meta_skill_type . bv) 0) 0) s)
              (write-long (ash (elt (_meta_skill_type . bv) 1) -1) s))
             ((and (class _meta_skill_type) (= (length (_meta_skill_type . bv)) 1)) ;; big1
              (write-long (elt (_meta_skill_type . bv) 0) s)
              (write-long (if (>= _meta_skill_type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _meta_skill_type s)(write-long (if (>= _meta_skill_type 0) 0 #xffffffff) s)))
     ;; int64 _meta_skill_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _meta_skill_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _meta_skill_id) (= (length (_meta_skill_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_meta_skill_id . bv) 0) 0) s)
              (write-long (ash (elt (_meta_skill_id . bv) 1) -1) s))
             ((and (class _meta_skill_id) (= (length (_meta_skill_id . bv)) 1)) ;; big1
              (write-long (elt (_meta_skill_id . bv) 0) s)
              (write-long (if (>= _meta_skill_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _meta_skill_id s)(write-long (if (>= _meta_skill_id 0) 0 #xffffffff) s)))
     ;; string[] _sensor_topics
     (write-long (length _sensor_topics) s)
     (dolist (elem _sensor_topics)
       (write-long (length elem) s) (princ elem s)
       )
     ;; uint64[] _sensor_value_sizes
     (write-long (length _sensor_value_sizes) s)
     (dotimes (i (length _sensor_value_sizes))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _sensor_value_sizes i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _sensor_value_sizes i)) (= (length ((elt _sensor_value_sizes i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _sensor_value_sizes i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _sensor_value_sizes i) . bv) 1) -1) s))
             ((and (class (elt _sensor_value_sizes i)) (= (length ((elt _sensor_value_sizes i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _sensor_value_sizes i) . bv) 0) s)
              (write-long (if (>= (elt _sensor_value_sizes i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _sensor_value_sizes i) s)(write-long (if (>= (elt _sensor_value_sizes i) 0) 0 #xffffffff) s)))
       )
     ;; float64[] _initial_sensor_values
     (write-long (length _initial_sensor_values) s)
     (dotimes (i (length _initial_sensor_values))
       (sys::poke (elt _initial_sensor_values i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int64 _traj_gen_type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _traj_gen_type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _traj_gen_type) (= (length (_traj_gen_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_traj_gen_type . bv) 0) 0) s)
              (write-long (ash (elt (_traj_gen_type . bv) 1) -1) s))
             ((and (class _traj_gen_type) (= (length (_traj_gen_type . bv)) 1)) ;; big1
              (write-long (elt (_traj_gen_type . bv) 0) s)
              (write-long (if (>= _traj_gen_type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _traj_gen_type s)(write-long (if (>= _traj_gen_type 0) 0 #xffffffff) s)))
     ;; uint64 _num_traj_gen_params
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num_traj_gen_params (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num_traj_gen_params) (= (length (_num_traj_gen_params . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num_traj_gen_params . bv) 0) 0) s)
              (write-long (ash (elt (_num_traj_gen_params . bv) 1) -1) s))
             ((and (class _num_traj_gen_params) (= (length (_num_traj_gen_params . bv)) 1)) ;; big1
              (write-long (elt (_num_traj_gen_params . bv) 0) s)
              (write-long (if (>= _num_traj_gen_params 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num_traj_gen_params s)(write-long (if (>= _num_traj_gen_params 0) 0 #xffffffff) s)))
     ;; float64[] _traj_gen_params
     (write-long (length _traj_gen_params) s)
     (dotimes (i (length _traj_gen_params))
       (sys::poke (elt _traj_gen_params i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int64 _feedback_controller_type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _feedback_controller_type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _feedback_controller_type) (= (length (_feedback_controller_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_feedback_controller_type . bv) 0) 0) s)
              (write-long (ash (elt (_feedback_controller_type . bv) 1) -1) s))
             ((and (class _feedback_controller_type) (= (length (_feedback_controller_type . bv)) 1)) ;; big1
              (write-long (elt (_feedback_controller_type . bv) 0) s)
              (write-long (if (>= _feedback_controller_type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _feedback_controller_type s)(write-long (if (>= _feedback_controller_type 0) 0 #xffffffff) s)))
     ;; uint64 _num_feedback_controller_params
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num_feedback_controller_params (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num_feedback_controller_params) (= (length (_num_feedback_controller_params . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num_feedback_controller_params . bv) 0) 0) s)
              (write-long (ash (elt (_num_feedback_controller_params . bv) 1) -1) s))
             ((and (class _num_feedback_controller_params) (= (length (_num_feedback_controller_params . bv)) 1)) ;; big1
              (write-long (elt (_num_feedback_controller_params . bv) 0) s)
              (write-long (if (>= _num_feedback_controller_params 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num_feedback_controller_params s)(write-long (if (>= _num_feedback_controller_params 0) 0 #xffffffff) s)))
     ;; float64[] _feedback_controller_params
     (write-long (length _feedback_controller_params) s)
     (dotimes (i (length _feedback_controller_params))
       (sys::poke (elt _feedback_controller_params i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int64 _termination_type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _termination_type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _termination_type) (= (length (_termination_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_termination_type . bv) 0) 0) s)
              (write-long (ash (elt (_termination_type . bv) 1) -1) s))
             ((and (class _termination_type) (= (length (_termination_type . bv)) 1)) ;; big1
              (write-long (elt (_termination_type . bv) 0) s)
              (write-long (if (>= _termination_type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _termination_type s)(write-long (if (>= _termination_type 0) 0 #xffffffff) s)))
     ;; uint64 _num_termination_params
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num_termination_params (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num_termination_params) (= (length (_num_termination_params . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num_termination_params . bv) 0) 0) s)
              (write-long (ash (elt (_num_termination_params . bv) 1) -1) s))
             ((and (class _num_termination_params) (= (length (_num_termination_params . bv)) 1)) ;; big1
              (write-long (elt (_num_termination_params . bv) 0) s)
              (write-long (if (>= _num_termination_params 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num_termination_params s)(write-long (if (>= _num_termination_params 0) 0 #xffffffff) s)))
     ;; float64[] _termination_params
     (write-long (length _termination_params) s)
     (dotimes (i (length _termination_params))
       (sys::poke (elt _termination_params i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int64 _timer_type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _timer_type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _timer_type) (= (length (_timer_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_timer_type . bv) 0) 0) s)
              (write-long (ash (elt (_timer_type . bv) 1) -1) s))
             ((and (class _timer_type) (= (length (_timer_type . bv)) 1)) ;; big1
              (write-long (elt (_timer_type . bv) 0) s)
              (write-long (if (>= _timer_type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _timer_type s)(write-long (if (>= _timer_type 0) 0 #xffffffff) s)))
     ;; uint64 _num_timer_params
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num_timer_params (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num_timer_params) (= (length (_num_timer_params . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num_timer_params . bv) 0) 0) s)
              (write-long (ash (elt (_num_timer_params . bv) 1) -1) s))
             ((and (class _num_timer_params) (= (length (_num_timer_params . bv)) 1)) ;; big1
              (write-long (elt (_num_timer_params . bv) 0) s)
              (write-long (if (>= _num_timer_params 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num_timer_params s)(write-long (if (>= _num_timer_params 0) 0 #xffffffff) s)))
     ;; float64[] _timer_params
     (write-long (length _timer_params) s)
     (dotimes (i (length _timer_params))
       (sys::poke (elt _timer_params i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _skill_type
#+(or :alpha :irix6 :x86_64)
      (setf _skill_type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _skill_type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _skill_description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _skill_description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _meta_skill_type
#+(or :alpha :irix6 :x86_64)
      (setf _meta_skill_type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _meta_skill_type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _meta_skill_id
#+(or :alpha :irix6 :x86_64)
      (setf _meta_skill_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _meta_skill_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string[] _sensor_topics
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sensor_topics (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _sensor_topics i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; uint64[] _sensor_value_sizes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sensor_value_sizes (instantiate integer-vector n))
     (dotimes (i n)
     
#+(or :alpha :irix6 :x86_64)
      (setf (elt _sensor_value_sizes i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _sensor_value_sizes i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; float64[] _initial_sensor_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _initial_sensor_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _initial_sensor_values i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int64 _traj_gen_type
#+(or :alpha :irix6 :x86_64)
      (setf _traj_gen_type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _traj_gen_type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _num_traj_gen_params
     
#+(or :alpha :irix6 :x86_64)
      (setf _num_traj_gen_params (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num_traj_gen_params (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64[] _traj_gen_params
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _traj_gen_params (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _traj_gen_params i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int64 _feedback_controller_type
#+(or :alpha :irix6 :x86_64)
      (setf _feedback_controller_type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _feedback_controller_type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _num_feedback_controller_params
     
#+(or :alpha :irix6 :x86_64)
      (setf _num_feedback_controller_params (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num_feedback_controller_params (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64[] _feedback_controller_params
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _feedback_controller_params (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _feedback_controller_params i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int64 _termination_type
#+(or :alpha :irix6 :x86_64)
      (setf _termination_type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _termination_type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _num_termination_params
     
#+(or :alpha :irix6 :x86_64)
      (setf _num_termination_params (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num_termination_params (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64[] _termination_params
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _termination_params (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _termination_params i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int64 _timer_type
#+(or :alpha :irix6 :x86_64)
      (setf _timer_type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _timer_type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _num_timer_params
     
#+(or :alpha :irix6 :x86_64)
      (setf _num_timer_params (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num_timer_params (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64[] _timer_params
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _timer_params (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _timer_params i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get franka_action_lib::ExecuteSkillGoal :md5sum-) "0a5a9702eeb075de3d4f7812c5b9bf0c")
(setf (get franka_action_lib::ExecuteSkillGoal :datatype-) "franka_action_lib/ExecuteSkillGoal")
(setf (get franka_action_lib::ExecuteSkillGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
int64 skill_type
string skill_description
int64 meta_skill_type
int64 meta_skill_id

# Sensor topic to subscribe to
string[] sensor_topics
uint64[] sensor_value_sizes
float64[] initial_sensor_values

# traj gen
int64 traj_gen_type
uint64 num_traj_gen_params
float64[] traj_gen_params

# fbc
int64 feedback_controller_type
uint64 num_feedback_controller_params
float64[] feedback_controller_params

# termination
int64 termination_type
uint64 num_termination_params
float64[] termination_params 

# timer
int64 timer_type
uint64 num_timer_params
float64[] timer_params

")



(provide :franka_action_lib/ExecuteSkillGoal "0a5a9702eeb075de3d4f7812c5b9bf0c")


