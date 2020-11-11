;; Auto-generated. Do not edit!


(when (boundp 'dynamo_planner::data_gen_msgs)
  (if (not (find-package "DYNAMO_PLANNER"))
    (make-package "DYNAMO_PLANNER"))
  (shadow 'data_gen_msgs (find-package "DYNAMO_PLANNER")))
(unless (find-package "DYNAMO_PLANNER::DATA_GEN_MSGS")
  (make-package "DYNAMO_PLANNER::DATA_GEN_MSGS"))

(in-package "ROS")
;;//! \htmlinclude data_gen_msgs.msg.html


(defclass dynamo_planner::data_gen_msgs
  :super ros::object
  :slots (_flag _filename _num_data ))

(defmethod dynamo_planner::data_gen_msgs
  (:init
   (&key
    ((:flag __flag) nil)
    ((:filename __filename) "")
    ((:num_data __num_data) 0.0)
    )
   (send-super :init)
   (setq _flag __flag)
   (setq _filename (string __filename))
   (setq _num_data (float __num_data))
   self)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:filename
   (&optional __filename)
   (if __filename (setq _filename __filename)) _filename)
  (:num_data
   (&optional __num_data)
   (if __num_data (setq _num_data __num_data)) _num_data)
  (:serialization-length
   ()
   (+
    ;; bool _flag
    1
    ;; string _filename
    4 (length _filename)
    ;; float64 _num_data
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _flag
       (if _flag (write-byte -1 s) (write-byte 0 s))
     ;; string _filename
       (write-long (length _filename) s) (princ _filename s)
     ;; float64 _num_data
       (sys::poke _num_data (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _flag
     (setq _flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _num_data
     (setq _num_data (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get dynamo_planner::data_gen_msgs :md5sum-) "5217da41250023fd463ea99124c27fdc")
(setf (get dynamo_planner::data_gen_msgs :datatype-) "dynamo_planner/data_gen_msgs")
(setf (get dynamo_planner::data_gen_msgs :definition-)
      "# bool flag string filename float64 num_data
bool flag
string filename
float64 num_data

")



(provide :dynamo_planner/data_gen_msgs "5217da41250023fd463ea99124c27fdc")


