;; Auto-generated. Do not edit!


(when (boundp 'mpc_controller::SE2Traj)
  (if (not (find-package "MPC_CONTROLLER"))
    (make-package "MPC_CONTROLLER"))
  (shadow 'SE2Traj (find-package "MPC_CONTROLLER")))
(unless (find-package "MPC_CONTROLLER::SE2TRAJ")
  (make-package "MPC_CONTROLLER::SE2TRAJ"))

(in-package "ROS")
;;//! \htmlinclude SE2Traj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass mpc_controller::SE2Traj
  :super ros::object
  :slots (_start_time _pos_pts _angle_pts _init_v _init_a _posT_pts _angleT_pts ))

(defmethod mpc_controller::SE2Traj
  (:init
   (&key
    ((:start_time __start_time) (instance ros::time :init))
    ((:pos_pts __pos_pts) ())
    ((:angle_pts __angle_pts) ())
    ((:init_v __init_v) (instance geometry_msgs::Vector3 :init))
    ((:init_a __init_a) (instance geometry_msgs::Vector3 :init))
    ((:posT_pts __posT_pts) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:angleT_pts __angleT_pts) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _start_time __start_time)
   (setq _pos_pts __pos_pts)
   (setq _angle_pts __angle_pts)
   (setq _init_v __init_v)
   (setq _init_a __init_a)
   (setq _posT_pts __posT_pts)
   (setq _angleT_pts __angleT_pts)
   self)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:pos_pts
   (&rest __pos_pts)
   (if (keywordp (car __pos_pts))
       (send* _pos_pts __pos_pts)
     (progn
       (if __pos_pts (setq _pos_pts (car __pos_pts)))
       _pos_pts)))
  (:angle_pts
   (&rest __angle_pts)
   (if (keywordp (car __angle_pts))
       (send* _angle_pts __angle_pts)
     (progn
       (if __angle_pts (setq _angle_pts (car __angle_pts)))
       _angle_pts)))
  (:init_v
   (&rest __init_v)
   (if (keywordp (car __init_v))
       (send* _init_v __init_v)
     (progn
       (if __init_v (setq _init_v (car __init_v)))
       _init_v)))
  (:init_a
   (&rest __init_a)
   (if (keywordp (car __init_a))
       (send* _init_a __init_a)
     (progn
       (if __init_a (setq _init_a (car __init_a)))
       _init_a)))
  (:posT_pts
   (&optional __posT_pts)
   (if __posT_pts (setq _posT_pts __posT_pts)) _posT_pts)
  (:angleT_pts
   (&optional __angleT_pts)
   (if __angleT_pts (setq _angleT_pts __angleT_pts)) _angleT_pts)
  (:serialization-length
   ()
   (+
    ;; time _start_time
    8
    ;; geometry_msgs/Point[] _pos_pts
    (apply #'+ (send-all _pos_pts :serialization-length)) 4
    ;; geometry_msgs/Point[] _angle_pts
    (apply #'+ (send-all _angle_pts :serialization-length)) 4
    ;; geometry_msgs/Vector3 _init_v
    (send _init_v :serialization-length)
    ;; geometry_msgs/Vector3 _init_a
    (send _init_a :serialization-length)
    ;; float64[] _posT_pts
    (* 8    (length _posT_pts)) 4
    ;; float64[] _angleT_pts
    (* 8    (length _angleT_pts)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; geometry_msgs/Point[] _pos_pts
     (write-long (length _pos_pts) s)
     (dolist (elem _pos_pts)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _angle_pts
     (write-long (length _angle_pts) s)
     (dolist (elem _angle_pts)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3 _init_v
       (send _init_v :serialize s)
     ;; geometry_msgs/Vector3 _init_a
       (send _init_a :serialize s)
     ;; float64[] _posT_pts
     (write-long (length _posT_pts) s)
     (dotimes (i (length _posT_pts))
       (sys::poke (elt _posT_pts i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _angleT_pts
     (write-long (length _angleT_pts) s)
     (dotimes (i (length _angleT_pts))
       (sys::poke (elt _angleT_pts i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point[] _pos_pts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos_pts (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _pos_pts)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _angle_pts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angle_pts (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _angle_pts)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3 _init_v
     (send _init_v :deserialize buf ptr-) (incf ptr- (send _init_v :serialization-length))
   ;; geometry_msgs/Vector3 _init_a
     (send _init_a :deserialize buf ptr-) (incf ptr- (send _init_a :serialization-length))
   ;; float64[] _posT_pts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _posT_pts (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _posT_pts i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _angleT_pts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angleT_pts (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _angleT_pts i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get mpc_controller::SE2Traj :md5sum-) "f921a322e80c694816eb9412daf79b62")
(setf (get mpc_controller::SE2Traj :datatype-) "mpc_controller/SE2Traj")
(setf (get mpc_controller::SE2Traj :definition-)
      "# MINCO trajectory

time start_time
geometry_msgs/Point[] pos_pts
geometry_msgs/Point[] angle_pts
geometry_msgs/Vector3 init_v
geometry_msgs/Vector3 init_a
float64[] posT_pts
float64[] angleT_pts

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :mpc_controller/SE2Traj "f921a322e80c694816eb9412daf79b62")


