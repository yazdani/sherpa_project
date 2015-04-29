;; Copyright (c) 2014, Fereshta Yazdani <yazdani@cs.uni-bremen.de>
;;; All rights reserved.
;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to 
;;;       endorse or promote products derived from this software without 
;;;       specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :startup-scenario)

(defvar *joint-states* nil
  "List of current joint states as published by /joint_states.")

(defvar *joint-states-subscriber* nil
  "Subscriber to /joint_states.")

(defparameter *bone-hash* (make-hash-table :test 'equal))
(defparameter *bone-list* (list
			    "pelvis_joint"
			    "l5_joint"
			    "l3_joint"
			    "t12_joint"
			    "t8_joint"
			    "neck_joint"
			    "head_joint"
			    "right_upper_arm_joint"
			    "right_lower_arm_joint"
			    "right_hand_joint"
			    "right_shoulder_joint"
			    "right_upper_leg_joint"
			    "right_lower_leg_joint"
			    "right_foot_joint"
			    "right_toe_joint"
			    "left_upper_arm_joint"
			    "left_lower_arm_joint"
			    "left_hand_joint"
			    "left_shoulder_joint"
			    "left_upper_leg_joint"
			    "left_lower_leg_joint"
			    "left_foot_joint"
			    "left_toe_joint"))

(defun pointing-direction ()
  (roslisp:ros-info (sherpa-spatial-relations) "POINTING GESTURE")
  (crs:prolog
   `(assert (btr:joint-state ?w human (("right_shoulder_joint_x" 0.06) ;;0.1
					("right_shoulder_joint_y" -0.25)  ;;0.0 0.40
					("right_shoulder_joint_z" 1.4)  ;;0.6 0.500
					("left_upper_arm_joint_x" 0.1)
					("left_upper_arm_joint_y" 3.0)
					("left_upper_arm_joint_z" -0.5)))))
  (init-joints)
  (execute-right-arm-trajectory (default-position-to-trajectory))
   (start-myros)
  ;; (let ((pose-arm (tf:make-pose-stamped
  ;;                  "/map"
  ;;                  0.0
  ;;                  (tf:make-3d-vector (+ (get-joint-value "right_hand_joint_x") 10) 
  ;;                                     (get-joint-value "right_hand_joint_y") 
  ;;                                     (get-joint-value "right_hand_joint_z"))
  ;;                  (tf:make-quaternion 0 0 0 1))))
  ;;   (format t "pose-arm is ~a ~%" pose-arm)
  ;;   pose-arm)
  )


(defun default-position-to-trajectory ()
  (roslisp:ros-info (sherpa-spatial-relations) "Position of human arm")
  (roslisp:make-msg "trajectory_msgs/JointTrajectory" 
                    (stamp header)
                    (roslisp:ros-time)
                    joint_names #("right_shoulder_joint_x" "right_shoulder_joint_y" "right_shoulder_joint_z")
                    points (vector
                            (roslisp:make-message
                             "trajectory_msgs/JointTrajectoryPoint"
                             positions #(1.5 0.0 -0.1)
                             velocities #(0 0 0)
                             accelerations #(0)
                             time_from_start 2.0
                             ))))

(defun execute-right-arm-trajectory (trajec)
  (roslisp:ros-info (sherpa-spatial-relations) "Execute-right")
  (let* ((act-cli (actionlib:make-action-client
                   "human/right_shoulder_joint_controller/follow_joint_trajectory"
                   "control_msgs/FollowJointTrajectoryAction"))
         (act-goal (actionlib:make-action-goal
                      act-cli
                     :trajectory 
                     (remove-trajectory-joints
                     #("right_shoulder_joint_x" "right_shoulder_joint_y" "right_shoulder_joint_z") 
                                  trajec :invert t))))

         (actionlib:wait-for-server act-cli)
         (actionlib:call-goal act-cli act-goal)))

;;;;;;;;;;;;;;;;;;;;SETTING SETUPS FOR POINTING INTO A DIRECTION;;;;;;;;;;;;;;;;;;;;;;
(defun seq-member (item sequence)
  (some (lambda (s)
          (equal item s))
        sequence))

(defun remove-trajectory-joints (joints trajectory &key invert)
  "Removes (or keeps) only the joints that are specified in
  `joints'. If `invert' is NIL, the named joints are removed,
  otherwise, they are kept."
  (roslisp:with-fields ((stamp (stamp header))
                        (joint-names joint_names)
                        (points points))
      trajectory
    (if (not invert)
        (roslisp:make-message
         "trajectory_msgs/JointTrajectory"
         (stamp header) stamp
         joint_names (remove-if (lambda (name)
                                  (seq-member name joints))
                                joint-names)
         points (map 'vector
                     (lambda (point)
                       (roslisp:with-fields (positions
                                             velocities
                                             accelerations
                                             time_from_start)
                           point
                         (roslisp:make-message
                          "trajectory_msgs/JointTrajectoryPoint"
                          positions (map 'vector #'identity
                                         (loop for n across joint-names
                                               for p across positions
                                               unless (seq-member n joints)
                                                 collecting p))
                          velocities (map 'vector #'identity
                                          (loop for n across joint-names
                                                for p across velocities
                                                unless (seq-member n joints)
                                                  collecting p))
                          accelerations (map 'vector #'identity
                                             (loop for n across joint-names
                                                   for p across accelerations
                                                   unless (seq-member n joints)
                                                     collecting p))
                          time_from_start time_from_start)))
                     points))
        (roslisp:make-message
         "trajectory_msgs/JointTrajectory"
         (stamp header) stamp
         joint_names (map 'vector #'identity
                          (loop for n across joint-names
                                when (seq-member n joints)
                                  collecting n))
         points (map 'vector
                     (lambda (point)
                       (roslisp:with-fields (positions time_from_start)
                           point
                         (roslisp:make-message
                          "trajectory_msgs/JointTrajectoryPoint"
                          positions (map 'vector #'identity
                                         (loop for n across joint-names
                                               for p across positions
                                               when (seq-member n joints)
                                                 collecting p))
                          time_from_start time_from_start)))
                     points)))))


;;; INIT HUMAN JOINTS ;;;
(defun init-joints ()
  (setf *joint-states-subscriber*
        (roslisp:subscribe "/human/joint_states"
                           "sensor_msgs/JointState"
                           #'joint-states-cb))
  (init-human))

(defun get-joint-value (str-name)
  (let* ((joint-states (sherpa-joint-states))
         (joint-state
           (nth (position str-name joint-states
                          :test (lambda (str-name state)
                                  (equal str-name (car state))))
              joint-states)))
    (cdr joint-state)))

(defun joint-states-cb (msg)
  (roslisp:with-fields (name position) msg
    (setf
     *joint-states*
     (loop for i from 0 below (length name)
           for n = (elt name i)
           for p = (elt position i)
           collect (cons n p)))))

(defun init-human ()
  "used to initialize the human controller for usage. creates advertisers for each joint and resets init position
"
    (roslisp:start-ros-node "human_teleop")
    (dolist (b *bone-list*) 
      (print 
       (format t "adding bone ~a" b)) 
      (add-bone b) 
      (change-bone-state b #(0 0 0)))
    (print "all bones initialized"))

(defun add-bone (name)
	"Used to add a bone to the hash map of available bones and creates a connection to the subscribed controller"
           (setf 
		(gethash name *bone-hash*) 
		(roslisp:advertise 
			(format nil "~a_controller/command" name) 
			"trajectory_msgs/JointTrajectory")))

(defun change-bone-state (name vector)
  " This function is used to change the state of a single bone"
  (let 
      ((pub  (gethash name *bone-hash*)) 
       (msg  (roslisp:make-msg "trajectory_msgs/JointTrajectory" 
			       joint_names (vector (format nil "~a_x" name) 
						   (format nil "~a_y" name) 
						   (format nil "~a_z" name))
			       points (vector
				       (roslisp:make-message
					"trajectory_msgs/JointTrajectoryPoint"
					positions vector
					velocities #(0 0 0)
					time_from_start 2.0)))))
    (roslisp:publish pub msg)))

(defun sherpa-joint-states ()
  *joint-states*)

(defun get-object-pose (obj-name)
  (let* ((lists (force-ll
                 (prolog `(and (bullet-world ?w)
                            (object-pose ?w ,obj-name ?pose)))))
         (list (car lists))
         (a-list (assoc '?pose list)))
    (cdr a-list)))

(defun get-object-name-from-type (obj-type)
  (let* ((lists (force-ll
                 (prolog `(and (bullet-world ?w)
                            (object-type ?w ?name ,obj-type)))))
         (list (car lists))
         (a-list (assoc '?name list)))
    
    (cdr a-list)))

(defun get-object-from-name (obj-name)
   (let* ((lists (force-ll
                 (prolog `(and (bullet-world ?w)
                            (%object ?w ,obj-name ?obj-ins)))))
         (list (car lists))
         (a-list (assoc '?obj-ins list)))
    (cdr a-list)))


(defun set-object-new-pose (pose obj-name)
  (let* ((vector (cl-transforms:origin pose))
         (vec-x  (cl-transforms:x vector))
         (vec-y  (cl-transforms:y vector))
         (vec-z   (cl-transforms:z vector))
         (height-z   (+ 2 vec-z))
         (quaternion (cl-transforms:orientation pose))
         (quat-x  (cl-transforms:x quaternion))
         (quat-y  (cl-transforms:y quaternion))
         (quat-z   (cl-transforms:z quaternion))
         (quat-w   (cl-transforms:w quaternion)))
    (cond ((eql obj-name 'quad)
           (prolog `(and 
                     (bullet-world ?w)
                     (assert 
                      (object-pose ?w ,obj-name 
                                   ((,vec-x ,vec-y ,height-z)(,quat-x ,quat-y ,quat-z ,quat-w))))))
           (format t "here1~%"))
          (t 
           (prolog `(and 
                     (bullet-world ?w)
                     (assert 
                      (object-pose ?w ,obj-name 
                                   ((,vec-x ,vec-y ,vec-z)(,quat-x ,quat-y ,quat-z ,quat-w))))))
           (format t "here2~%")))))

(defun visualize-in-x (offset)
  (format t "visualizing in x direction~%")
  (let*((adder 0)
        (caller 0))
    (prolog `(and (bullet-world ?w)
                  (assert (object ?w mesh sphere0 
                                  ((,(cl-transforms:x  *origin-pose*) 
                                     ,(cl-transforms:y *origin-pose*)
                                     ,(cl-transforms:z *origin-pose*))
                                   (,(cl-transforms:x (cl-transforms:x (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*))))) 
                                     ,(cl-transforms:y (cl-transforms:y (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*))))) 
                                     ,(cl-transforms:z (cl-transforms:z (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                                     ,(cl-transforms:w (cl-transforms:w (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))))
                                  :mesh cognitive-reasoning::sphere 
                                  :mass 0.2 :color (0 1 0) :scale 3.0))))
    (cond ((< offset (cl-transforms:x *origin-pose*))
           (loop until (not (equal nil (prolog `(and 
                                                 (bullet-world ?w)
                                                 (contact ?w ,
                                                          (read-from-string 
                                                           (format nil "sphere~a" caller)) ?sem)))))
                 do
                    (setf adder (- adder 1))
                    (setf caller (+ caller 1))
                    (add-sphere (read-from-string (format nil "sphere~a" caller)) 
                                (cl-transforms:make-pose 
                                 (cl-transforms:make-3d-vector 
                                  (+ (cl-transforms:x *origin-pose*) adder) 
                                  (cl-transforms:y *origin-pose*) 
                                  (cl-transforms:z *origin-pose*))
                                 (cl-transforms:make-quaternion
                                  (cl-transforms:x (cl-transforms:x (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                                  (cl-transforms:y (cl-transforms:y (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                                  (cl-transforms:z (cl-transforms:z (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                                  (cl-transforms:w (cl-transforms:w (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*))))))))))
          ((> offset (cl-transforms:x *origin-pose*))
           (loop until (not (equal nil (prolog `(and 
                                                 (bullet-world ?w)
                                                 (contact ?w ,
                                                          (read-from-string 
                                                           (format nil "sphere~a" caller)) ?sem)))))
                 do
                    (setf adder (+ adder 1))
                    (setf caller (+ caller 1))
                    (add-sphere (read-from-string (format nil "sphere~a" caller)) 
                                (cl-transforms:make-pose 
                                 (cl-transforms:make-3d-vector 
                                  (+ (cl-transforms:x *origin-pose*) adder) 
                                  (cl-transforms:y *origin-pose*) 
                                  (cl-transforms:z *origin-pose*))
                                 (cl-transforms:make-quaternion
                                  (cl-transforms:x (cl-transforms:x (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                                  (cl-transforms:y (cl-transforms:y (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                                  (cl-transforms:z (cl-transforms:z (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                                  (cl-transforms:w (cl-transforms:w (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))))))))))



(defun visualize-in-y (offset)
  (let*((adder 0)
        (caller 0)
        (iterator (cl-transforms:y *origin-pose*)))
    (prolog `(and 
              (bullet-world ?w)
              (assert (object ?w mesh sphere0 
                              ((,(cl-transforms:x  *origin-pose*) 
                                 ,(cl-transforms:y *origin-pose*)
                                 ,(cl-transforms:z *origin-pose*))
                               (,(cl-transforms:x (cl-transforms:x (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*))))) 
                                 ,(cl-transforms:y (cl-transforms:y (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*))))) 
                                 ,(cl-transforms:z (cl-transforms:z (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                                 ,(cl-transforms:w (cl-transforms:w (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))))
                              :mesh cognitive-reasoning::sphere 
                              :mass 0.2 :color (0 1 0) :scale 3.0))))
    (if (> (cl-transforms:y (cadr (assoc 'offset *visualize-list*))) 0)
        (loop until (<= offset iterator)
              do
                 (setf adder (+ adder 1))
                 (setf caller (+ caller 1))
                 (setf iterator (+ iterator 1))
                 (add-sphere (read-from-string (format nil "sphere~a" caller)) 
                             (cl-transforms:make-pose 
                              (cl-transforms:make-3d-vector 
                               (cl-transforms:x *origin-pose*) 
                               (+ (cl-transforms:y *origin-pose*) adder)
                               (cl-transforms:z *origin-pose*))
                              (cl-transforms:make-quaternion
                               (cl-transforms:x (cl-transforms:x (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                               (cl-transforms:y (cl-transforms:y (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                               (cl-transforms:z (cl-transforms:z (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                               (cl-transforms:w (cl-transforms:w (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*))))))))))
    (if (< (cl-transforms:y (cadr (assoc 'offset *visualize-list*))) 0 )
        (loop until (>= offset iterator) 
              do
                 (setf adder (- adder 1))
                 (setf caller (+ caller 1))
                 (setf iterator (- iterator 1))
                 (add-sphere (read-from-string (format nil "sphere~a" caller)) 
                             (cl-transforms:make-pose 
                              (cl-transforms:make-3d-vector 
                               (cl-transforms:x *origin-pose*) 
                               (+ (cl-transforms:y *origin-pose*) adder)
                               (cl-transforms:z *origin-pose*))
                              (cl-transforms:make-quaternion
                               (cl-transforms:x (cl-transforms:x (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                               (cl-transforms:y (cl-transforms:y (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                               (cl-transforms:z (cl-transforms:z (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                               (cl-transforms:w (cl-transforms:w (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*))))))))))))


(defun visualize-in-z (offset)
  (let*((adder 0)
        (caller 0)
        (iterator (cl-transforms:z *origin-pose*)))
    (prolog `(and (bullet-world ?w)
                  (assert (object ?w mesh sphere0 
                                  ((,(cl-transforms:x  *origin-pose*) 
                                     ,(cl-transforms:y *origin-pose*)
                                     ,(cl-transforms:z *origin-pose*))
                                   (,(cl-transforms:x (cl-transforms:x (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*))))) 
                                     ,(cl-transforms:y (cl-transforms:y (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*))))) 
                                     ,(cl-transforms:z (cl-transforms:z (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))q
                                     ,(cl-transforms:w (cl-transforms:w (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))))
                                  :mesh cognitive-reasoning::sphere 
                                  :mass 0.2 :color (0 1 0) :scale 3.0))))
    (if (> (cl-transforms:z (cadr (assoc 'offset *visualize-list*))) 0)
        (loop until  (<= offset iterator)  
              do
                 (setf adder (+ adder 1))
                 (setf caller (+ caller 1))
                 (setf iterator (+ iterator 1))
                 (add-sphere (read-from-string (format nil "sphere~a" caller)) 
                         (cl-transforms:make-pose 
                          (cl-transforms:make-3d-vector 
                           (cl-transforms:x *origin-pose*)
                           (cl-transforms:y *origin-pose*)  
                           (+ (cl-transforms:z *origin-pose*) adder))
                       (cl-transforms:make-quaternion
                           (cl-transforms:x (cl-transforms:x (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                           (cl-transforms:y (cl-transforms:y (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                           (cl-transforms:z (cl-transforms:z (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                           (cl-transforms:w (cl-transforms:w (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*))))))))))
    (if (< (cl-transforms:z (cadr (assoc 'offset *visualize-list*))) 0 )
        (loop until (>= offset iterator)
          do
             (setf adder (- adder 1))
             (setf caller (+ caller 1))
             (setf iterator (- iterator 1))
             (add-sphere (read-from-string (format nil "sphere~a" caller)) 
                         (cl-transforms:make-pose 
                          (cl-transforms:make-3d-vector 
                           (cl-transforms:x *origin-pose*) 
                           (cl-transforms:y  *origin-pose*) 
                           (+ (cl-transforms:z  *origin-pose*) adder))
                          (cl-transforms:make-quaternion
                           (cl-transforms:x (cl-transforms:x (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                           (cl-transforms:y (cl-transforms:y (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                           (cl-transforms:z (cl-transforms:z (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))
                           (cl-transforms:w (cl-transforms:w (cl-transforms:orientation (cadr (assoc 'loc *visualize-list*)))))))))))) 