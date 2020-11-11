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
  :slots (_x _y _yaw _controlA _controlB _control _controlYAW _duration _pre_x _pre_y _pre_yaw _pre_controlA _pre_controlB _flag _sub_flag _index ))

(defmethod dynamo_planner::custom_states_msgs
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:yaw __yaw) 0.0)
    ((:controlA __controlA) 0.0)
    ((:controlB __controlB) 0.0)
    ((:control __control) 0.0)
    ((:controlYAW __controlYAW) 0.0)
    ((:duration __duration) 0.0)
    ((:pre_x __pre_x) 0.0)
    ((:pre_y __pre_y) 0.0)
    ((:pre_yaw __pre_yaw) 0.0)
    ((:pre_controlA __pre_controlA) 0.0)
    ((:pre_controlB __pre_controlB) 0.0)
    ((:flag __flag) nil)
    ((:sub_flag __sub_flag) nil)
    ((:index __index) 0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _yaw (float __yaw))
   (setq _controlA (float __controlA))
   (setq _controlB (float __controlB))
   (setq _control (float __control))
   (setq _controlYAW (float __controlYAW))
   (setq _duration (float __duration))
   (setq _pre_x (float __pre_x))
   (setq _pre_y (float __pre_y))
   (setq _pre_yaw (float __pre_yaw))
   (setq _pre_controlA (float __pre_controlA))
   (setq _pre_controlB (float __pre_controlB))
   (setq _flag __flag)
   (setq _sub_flag __sub_flag)
   (setq _index (round __index))
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
  (:controlA
   (&optional __controlA)
   (if __controlA (setq _controlA __controlA)) _controlA)
  (:controlB
   (&optional __controlB)
   (if __controlB (setq _controlB __controlB)) _controlB)
  (:control
   (&optional __control)
   (if __control (setq _control __control)) _control)
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
  (:pre_controlA
   (&optional __pre_controlA)
   (if __pre_controlA (setq _pre_controlA __pre_controlA)) _pre_controlA)
  (:pre_controlB
   (&optional __pre_controlB)
   (if __pre_controlB (setq _pre_controlB __pre_controlB)) _pre_controlB)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:sub_flag
   (&optional __sub_flag)
   (if __sub_flag (setq _sub_flag __sub_flag)) _sub_flag)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _yaw
    8
    ;; float64 _controlA
    8
    ;; float64 _controlB
    8
    ;; float64 _control
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
    ;; float64 _pre_controlA
    8
    ;; float64 _pre_controlB
    8
    ;; bool _flag
    1
    ;; bool _sub_flag
    1
    ;; int8 _index
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
     ;; float64 _controlA
       (sys::poke _controlA (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _controlB
       (sys::poke _controlB (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _control
       (sys::poke _control (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
     ;; float64 _pre_controlA
       (sys::poke _pre_controlA (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pre_controlB
       (sys::poke _pre_controlB (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _flag
       (if _flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _sub_flag
       (if _sub_flag (write-byte -1 s) (write-byte 0 s))
     ;; int8 _index
       (write-byte _index s)
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
   ;; float64 _controlA
     (setq _controlA (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _controlB
     (setq _controlB (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _control
     (setq _control (sys::peek buf ptr- :double)) (incf ptr- 8)
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
   ;; float64 _pre_controlA
     (setq _pre_controlA (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pre_controlB
     (setq _pre_controlB (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _flag
     (setq _flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _sub_flag
     (setq _sub_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _index
     (setq _index (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _index 127) (setq _index (- _index 256)))
   ;;
   self)
  )

(setf (get dynamo_planner::custom_states_msgs :md5sum-) "687ab8d67518730041fc3c6e12a166b7")
(setf (get dynamo_planner::custom_states_msgs :datatype-) "dynamo_planner/custom_states_msgs")
(setf (get dynamo_planner::custom_states_msgs :definition-)
      "# double type x y yaw control duration
float64 x
float64 y
float64 yaw

float64 controlA
float64 controlB
float64 control
float64 controlYAW

float64 duration

float64 pre_x
float64 pre_y
float64 pre_yaw
float64 pre_controlA
float64 pre_controlB

bool flag
bool sub_flag

int8 index

")



(provide :dynamo_planner/custom_states_msgs "687ab8d67518730041fc3c6e12a166b7")


