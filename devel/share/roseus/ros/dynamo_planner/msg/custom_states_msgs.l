;; Auto-generated. Do not edit!


(when (boundp 'dynamo_planner::custom_states_msgs)
  (if (not (find-package "DYNAMO_PLANNER"))
    (make-package "DYNAMO_PLANNER"))
  (shadow 'custom_states_msgs (find-package "DYNAMO_PLANNER")))
(unless (find-package "DYNAMO_PLANNER::CUSTOM_STATES_MSGS")
  (make-package "DYNAMO_PLANNER::CUSTOM_STATES_MSGS"))

(in-package "ROS")
;;//! \htmlinclude custom_states_msgs.msg.html


(defclass dynamo_planner::custom_states_msgs
  :super ros::object
  :slots (_x _y _yaw _controlX _controlY _controlYAW _duration _pre_x _pre_y _pre_yaw _flag _tf_flag _diff_x _diff_y _diff_yaw ))

(defmethod dynamo_planner::custom_states_msgs
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:yaw __yaw) 0.0)
    ((:controlX __controlX) 0.0)
    ((:controlY __controlY) 0.0)
    ((:controlYAW __controlYAW) 0.0)
    ((:duration __duration) 0.0)
    ((:pre_x __pre_x) 0.0)
    ((:pre_y __pre_y) 0.0)
    ((:pre_yaw __pre_yaw) 0.0)
    ((:flag __flag) nil)
    ((:tf_flag __tf_flag) nil)
    ((:diff_x __diff_x) nil)
    ((:diff_y __diff_y) nil)
    ((:diff_yaw __diff_yaw) nil)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _yaw (float __yaw))
   (setq _controlX (float __controlX))
   (setq _controlY (float __controlY))
   (setq _controlYAW (float __controlYAW))
   (setq _duration (float __duration))
   (setq _pre_x (float __pre_x))
   (setq _pre_y (float __pre_y))
   (setq _pre_yaw (float __pre_yaw))
   (setq _flag __flag)
   (setq _tf_flag __tf_flag)
   (setq _diff_x __diff_x)
   (setq _diff_y __diff_y)
   (setq _diff_yaw __diff_yaw)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:controlX
   (&optional __controlX)
   (if __controlX (setq _controlX __controlX)) _controlX)
  (:controlY
   (&optional __controlY)
   (if __controlY (setq _controlY __controlY)) _controlY)
  (:controlYAW
   (&optional __controlYAW)
   (if __controlYAW (setq _controlYAW __controlYAW)) _controlYAW)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:pre_x
   (&optional __pre_x)
   (if __pre_x (setq _pre_x __pre_x)) _pre_x)
  (:pre_y
   (&optional __pre_y)
   (if __pre_y (setq _pre_y __pre_y)) _pre_y)
  (:pre_yaw
   (&optional __pre_yaw)
   (if __pre_yaw (setq _pre_yaw __pre_yaw)) _pre_yaw)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:tf_flag
   (&optional __tf_flag)
   (if __tf_flag (setq _tf_flag __tf_flag)) _tf_flag)
  (:diff_x
   (&optional __diff_x)
   (if __diff_x (setq _diff_x __diff_x)) _diff_x)
  (:diff_y
   (&optional __diff_y)
   (if __diff_y (setq _diff_y __diff_y)) _diff_y)
  (:diff_yaw
   (&optional __diff_yaw)
   (if __diff_yaw (setq _diff_yaw __diff_yaw)) _diff_yaw)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _yaw
    8
    ;; float64 _controlX
    8
    ;; float64 _controlY
    8
    ;; float64 _controlYAW
    8
    ;; float64 _duration
    8
    ;; float64 _pre_x
    8
    ;; float64 _pre_y
    8
    ;; float64 _pre_yaw
    8
    ;; bool _flag
    1
    ;; bool _tf_flag
    1
    ;; bool _diff_x
    1
    ;; bool _diff_y
    1
    ;; bool _diff_yaw
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _controlX
       (sys::poke _controlX (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _controlY
       (sys::poke _controlY (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _controlYAW
       (sys::poke _controlYAW (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pre_x
       (sys::poke _pre_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pre_y
       (sys::poke _pre_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pre_yaw
       (sys::poke _pre_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _flag
       (if _flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _tf_flag
       (if _tf_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _diff_x
       (if _diff_x (write-byte -1 s) (write-byte 0 s))
     ;; bool _diff_y
       (if _diff_y (write-byte -1 s) (write-byte 0 s))
     ;; bool _diff_yaw
       (if _diff_yaw (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _controlX
     (setq _controlX (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _controlY
     (setq _controlY (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _controlYAW
     (setq _controlYAW (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _duration
     (setq _duration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pre_x
     (setq _pre_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pre_y
     (setq _pre_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pre_yaw
     (setq _pre_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _flag
     (setq _flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _tf_flag
     (setq _tf_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _diff_x
     (setq _diff_x (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _diff_y
     (setq _diff_y (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _diff_yaw
     (setq _diff_yaw (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get dynamo_planner::custom_states_msgs :md5sum-) "3b52bf44a08210bd7d5f8e100b5f756a")
(setf (get dynamo_planner::custom_states_msgs :datatype-) "dynamo_planner/custom_states_msgs")
(setf (get dynamo_planner::custom_states_msgs :definition-)
      "# double type x y yaw control duration
float64 x
float64 y
float64 yaw

float64 controlX
float64 controlY
float64 controlYAW

float64 duration

float64 pre_x
float64 pre_y
float64 pre_yaw

bool flag
bool tf_flag

bool diff_x
bool diff_y
bool diff_yaw

")



(provide :dynamo_planner/custom_states_msgs "3b52bf44a08210bd7d5f8e100b5f756a")


