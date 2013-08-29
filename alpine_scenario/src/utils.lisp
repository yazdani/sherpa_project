(in-package :alpine_scenario)

(defvar *quadrotor-pose* (make-fluent :name :quadrotor-pose) "current pose of quadrotor")

(defun init-ros-quadrotor (name)
"subscribes to topics for a quadrotor and binds callbacks. `name' specifies the name of the quadrotor."
(setf *quadrotor* (subscribe (format nil "slam_out_pose" name)
				  "geometry_msgs/PoseStamped"
				  #'pose-cb)))

(defun pose-cb (msg)
"Callback for pose values"
(setf (value *quadrotor-pose*) msg))