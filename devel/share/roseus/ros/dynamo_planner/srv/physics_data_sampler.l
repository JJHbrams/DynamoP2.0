;; Auto-generated. Do not edit!


(when (boundp 'dynamo_planner::physics_data_sampler)
  (if (not (find-package "DYNAMO_PLANNER"))
    (make-package "DYNAMO_PLANNER"))
  (shadow 'physics_data_sampler (find-package "DYNAMO_PLANNER")))
(unless (find-package "DYNAMO_PLANNER::PHYSICS_DATA_SAMPLER")
  (make-package "DYNAMO_PLANNER::PHYSICS_DATA_SAMPLER"))
(unless (find-package "DYNAMO_PLANNER::PHYSICS_DATA_SAMPLERREQUEST")
  (make-package "DYNAMO_PLANNER::PHYSICS_DATA_SAMPLERREQUEST"))
(unless (find-package "DYNAMO_PLANNER::PHYSICS_DATA_SAMPLERRESPONSE")
  (make-package "DYNAMO_PLANNER::PHYSICS_DATA_SAMPLERRESPONSE"))

(in-package "ROS")





(defclass dynamo_planner::physics_data_samplerRequest
  :super ros::object
  :slots (_NUM_STEP ))

(defmethod dynamo_planner::physics_data_samplerRequest
  (:init
   (&key
    ((:NUM_STEP __NUM_STEP) 0)
    )
   (send-super :init)
   (setq _NUM_STEP (round __NUM_STEP))
   self)
  (:NUM_STEP
   (&optional __NUM_STEP)
   (if __NUM_STEP (setq _NUM_STEP __NUM_STEP)) _NUM_STEP)
  (:serialization-length
   ()
   (+
    ;; int64 _NUM_STEP
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _NUM_STEP
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _NUM_STEP (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _NUM_STEP) (= (length (_NUM_STEP . bv)) 2)) ;; bignum
              (write-long (ash (elt (_NUM_STEP . bv) 0) 0) s)
              (write-long (ash (elt (_NUM_STEP . bv) 1) -1) s))
             ((and (class _NUM_STEP) (= (length (_NUM_STEP . bv)) 1)) ;; big1
              (write-long (elt (_NUM_STEP . bv) 0) s)
              (write-long (if (>= _NUM_STEP 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _NUM_STEP s)(write-long (if (>= _NUM_STEP 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _NUM_STEP
#+(or :alpha :irix6 :x86_64)
      (setf _NUM_STEP (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _NUM_STEP (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass dynamo_planner::physics_data_samplerResponse
  :super ros::object
  :slots (_result ))

(defmethod dynamo_planner::physics_data_samplerResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dynamo_planner::physics_data_sampler
  :super ros::object
  :slots ())

(setf (get dynamo_planner::physics_data_sampler :md5sum-) "d13b09ac032499e03ad2334da1a598d0")
(setf (get dynamo_planner::physics_data_sampler :datatype-) "dynamo_planner/physics_data_sampler")
(setf (get dynamo_planner::physics_data_sampler :request) dynamo_planner::physics_data_samplerRequest)
(setf (get dynamo_planner::physics_data_sampler :response) dynamo_planner::physics_data_samplerResponse)

(defmethod dynamo_planner::physics_data_samplerRequest
  (:response () (instance dynamo_planner::physics_data_samplerResponse :init)))

(setf (get dynamo_planner::physics_data_samplerRequest :md5sum-) "d13b09ac032499e03ad2334da1a598d0")
(setf (get dynamo_planner::physics_data_samplerRequest :datatype-) "dynamo_planner/physics_data_samplerRequest")
(setf (get dynamo_planner::physics_data_samplerRequest :definition-)
      "
int64 NUM_STEP
---

bool result

")

(setf (get dynamo_planner::physics_data_samplerResponse :md5sum-) "d13b09ac032499e03ad2334da1a598d0")
(setf (get dynamo_planner::physics_data_samplerResponse :datatype-) "dynamo_planner/physics_data_samplerResponse")
(setf (get dynamo_planner::physics_data_samplerResponse :definition-)
      "
int64 NUM_STEP
---

bool result

")



(provide :dynamo_planner/physics_data_sampler "d13b09ac032499e03ad2334da1a598d0")


