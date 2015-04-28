;;; Copyright (c) 2014, Fereshta Yazdani <yazdani@cs.uni-bremen.de>
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
(defparameter *transform-listener* (make-instance 'cl-tf:transform-listener))
(defparameter *result-subscriber* nil)
(defparameter *stored-result* nil)
(defparameter *act-desig* nil)
(defparameter *agent-pose* nil)
(defvar *visualize-list* nil)
(defvar *switcher* nil)
(defvar *offset-checker* nil)

(defclass command-result ()
  ((content :reader content :initarg :content)
   (time-received :reader time-received :initarg :time-received)))

(defun start-world-with-robots ()
  (roslisp:ros-info (sherpa-spatial-relations) "START WORLD AND ROBOTS")
  (location-costmap::location-costmap-vis-init)
  (setf *list* nil)
  (let*((genius-urdf (cl-urdf:parse-urdf (roslisp:get-param "human/robot_description")))
        (quad-urdf (cl-urdf:parse-urdf (roslisp:get-param "red_hawk/robot_description")))
         (sem-urdf (cl-urdf:parse-urdf (roslisp:get-param "robot_description")))
         ;; (rover-urdf (cl-urdf:parse-urdf (roslisp:get-param "rover/robot_description")))
         )
    (setf *list*
          (car 
           (btr::force-ll
            (btr::prolog
             `(and
               (clear-bullet-world)
               (bullet-world ?w)
               (robot ?robot)
               (assert (object ?w static-plane floor ((0 0 0) (0 0 0 1))
                               :normal (0 0 1) :constant 0 :disable-collisions-with (?robot)))
               (debug-window ?w)
               (assert (object ?w semantic-map sem-map ((0 0 0) (0 0 0 1)) :urdf ,sem-urdf))
               (assert (object ?w urdf human ((-75.89979 -75.41413 29.02028)(0 0.0 -2 1)) :urdf ,genius-urdf))
               (assert (object ?w urdf quadrotor ((-81.73 -82.87 25.82)(0 0 0 1)) :urdf ,quad-urdf))
               (robot quadrotor)
               (robot human)
               ;; (assert (object ?w urdf quadrotor ((-1 -2 0.2)(0 0 1 1)) :urdf ,quad-urdf))
 
	       ;; (assert (object ?w urdf rover ((1 3 0) (0 0 0 1)) :urdf ,rover-urdf))
         )))))))

(defun init-base ()
  (format t "inside init-base~%")
  (setf *result-subscriber*
        (roslisp:subscribe "/interpreted_command"
                    "language_interpreter/connect"
                    #'cb-result)))

(defun cb-result (msg)
  (format t "inside cb-result~%")
  (setf *stored-result*
        (make-instance 'command-result
                       :content msg
                       :time-received (roslisp:ros-time))))

;;TODO: CHECKING IF SELECTION CONTAINS TWO STRINGS
 (defun command-into-designator ()
 ;  (init-base)
   (cond ((equal nil *stored-result*) (format t "Ain't no publisher~%"))
         (t
          (let*((value (content *stored-result*))
                (command (read-from-string (LANGUAGE_INTERPRETER-MSG:COMMAND value)))
                (interpretation (read-from-string 
                                 (car 
                                  (split-sequence:split-sequence #\( (LANGUAGE_INTERPRETER-MSG:INTERPRETATION value)))))
                (selection (split-sequence:split-sequence #\Space (LANGUAGE_INTERPRETER-MSG:SELECTION value)))
                (selection2 (read-from-string (concatenate 'string (car selection) "_" (second selection))))
                                        ;  (interpret (read-from-string (car (split-sequence:split-sequence #\( interpreted))))
                                        ;(command (read-from-string(LANGUAGE_INTERPRETER-MSG:COMMAND value)))
                (gesture00 (LANGUAGE_INTERPRETER-MSG:VEC value))
                (value-x (GEOMETRY_MSGS-MSG:X gesture00))
                (value-y (GEOMETRY_MSGS-MSG:Y gesture00))
                (value-z (GEOMETRY_MSGS-MSG:Z gesture00))
                (default (LANGUAGE_INTERPRETER-MSG:OFFSET value))
                (off-x (GEOMETRY_MSGS-MSG:X default))
                (off-y (GEOMETRY_MSGS-MSG:Y default))
                (off-z (GEOMETRY_MSGS-MSG:Z default)))
            (format t "tester~%")
            (checking-offset off-x off-y off-z)
            (cond ((equal value-x 0.0d0)
                   (equal value-y 0.0d0)
                   (equal value-z 0.0d0)
                   (setf *switcher* 1)
                   (setf gesture (cl-transforms:make-pose (cl-transforms:make-3d-vector 
                                                           (+ (- 81.73) off-x)
                                                           (+ (- 82.87) off-y)
                                                           (+ 25.82 off-z))
                                                          (cl-transforms:make-quaternion 0 0 0 1))))
                  ;;(setf gesture (tf-pose-of-agent (cl-transforms:make-3d-vector off-x off-y off-z))))
                  (t 
                   ;; (setf gesture (cl-transforms:make-pose (cl-transforms:make-3d-vector (+ value-x off-x) (+ value-y off-y) (+ value-z off-z))(cl-transforms:make-quaternion 0 0 0 1)))
                   (setf *switcher* 0)
                   (setf gesture (cl-transforms:make-pose (cl-transforms:make-3d-vector 
                                                           (+ (- 75.89979) (+ value-x off-x)) 
                                                           (+ (- 75.41413) (+ value-y off-y))
                                                           (+  29.02028 (+ value-z off-z)))
                                                          (cl-transforms:make-quaternion 0 0 0 1)))
                   ;; (setf *switcher* 0)
                   ))
            (setf def (cl-transforms:make-3d-vector off-x off-y off-z))
            (setf *visualize-list* `((offset ,def)(loc ,gesture)))
            (setf *act-desig* (make-designator 'action `((command_type ,command)
                                                         (action_type ,interpretation)
                                                         (offset ,def)
                                                         (agent ,selection2)
                                                         (target ,(make-designator 'location `((loc ,gesture))))))))
          
          *act-desig*
          )))

;; (defun get-tf-pose (robot)
;; (format t "TODO: get-tf-pose~%"))

(defun checking-offset (x y z)
  (format t "x y z ~a ~a ~a ~%"x y z)
  (cond ((not (equal x 0.0d0))(setf *offset-checker* 'x))
        ((not (equal y 0.0d0))(setf *offset-checker* 'y))
        (t
        (setf *offset-checker* 'z))))


 (defun tf-pose-of-agent (pose)
  (let* ((intern (tf:lookup-transform *transform-listener* :time 0.0 :source-frame "base_footprint" :target-frame "map"))
         (rob-pose (cl-transforms:transform->pose intern))
         (pose-x (+ (cl-transforms:x pose) (cl-transforms:x (cl-transforms:origin rob-pose))))
         (pose-y (+ (cl-transforms:y pose) (cl-transforms:y (cl-transforms:origin rob-pose))))
         (pose-z (+ (cl-transforms:z pose) (cl-transforms:z (cl-transforms:origin rob-pose))))
         (ori (cl-transforms:orientation rob-pose))
         (robot-pose (cl-transforms:make-pose (cl-transforms:make-3d-vector pose-x pose-y pose-z) ori)))
    (setf *agent-pose* robot-pose))
  *agent-pose*)

(defun visualize-pointing ()
  (format t "visualizing action designator~%")
  (let* ((offset-vec (cadr (assoc 'offset *visualize-list*)))
         (loc-pose (cadr (assoc 'loc *visualize-list*)))
         (vec (cl-transforms:origin loc-pose))
         (ori (cl-transforms:orientation loc-pose))
         (sum-x (+ (cl-transforms:x vec) (cl-transforms:x offset-vec)))
         (sum-y (+ (cl-transforms:y vec) (cl-transforms:y offset-vec)))
         (sum-z (+ (cl-transforms:z vec) (cl-transforms:z offset-vec))))
       ;  (new-vec (cl-transforms:make-3d-vector sum-x sum-y sum-z)))
    (cond ((string-equal *offset-checker* 'z)
           (visualize-in-z loc-pose sum-z)
          ((string-equal *offset-checker* 'y)
           (visualize-in-y loc-pose sum-y)
          (t
           (visualize-in-x loc-pose sum-x)))))))

(defun visualize-in-x (vec offset)
  (format t "visualizing in x direction~%")
  (let*((adder 0)
	(caller 0))
    (prolog `(and (bullet-world ?w)
		  (assert (object ?w mesh sphere0 
				  ((,(cl-transforms:x (cl-transforms:origin vec)) 
				     ,(cl-transforms:y (cl-transforms:origin vec)) 
				     ,(cl-transforms:z (cl-transforms:origin vec)))
				   (,(cl-transforms:x (cl-transforms:orientation vec)) 
				     ,(cl-transforms:y (cl-transforms:orientation vec)) 
				     ,(cl-transforms:z (cl-transforms:orientation vec))
				     ,(cl-transforms:w (cl-transforms:orientation vec))))
				  :mesh cognitive-reasoning::sphere 
				  :mass 0.2 :color (0 1 0) :scale 3.0))))
    (cond ((< offset (cl-transforms:x (cl-transforms:orientation vec)))
	   (loop until (not (equal nil (prolog `(and 
						 (bullet-world ?w)
						 (contact ?w ,
							  (read-from-string 
							   (format nil "sphere~a" caller)) ?sem)))))
	      do
		(setf adder (- adder1 1))
		(setf caller (+ caller 1))
		(add-sphere (read-from-string (format nil "sphere~a" caller)) 
			    (cl-transforms:make-pose 
			     (cl-transforms:make-3d-vector 
			      (+ (cl-transforms:x (cl-transforms:origin vec)) adder) 
			      (cl-transforms:y (cl-transforms:origin vec)) 
			      (cl-transforms:z (cl-transforms:origin vec)))
			     (cl-transforms:make-quaternion
			      (cl-transforms:x (cl-transforms:orientation vec))
			      (cl-transforms:y (cl-transforms:orientation vec))
			      (cl-transforms:z (cl-transforms:orientation vec))
			      (cl-transforms:w (cl-transforms:orientation vec)))))))
	  ((> offset (cl-transforms:x (cl-transforms:orientation vec)))
	   (loop until (not (equal nil (prolog `(and 
						 (bullet-world ?w)
						 (contact ?w ,
							  (read-from-string 
							   (format nil "sphere~a" caller)) ?sem)))))
	      do
		(setf adder (+ adder1 1))
		(setf caller (+ caller 1))
		(add-sphere (read-from-string (format nil "sphere~a" caller)) 
			    (cl-transforms:make-pose 
			     (cl-transforms:make-3d-vector 
			      (+ (cl-transforms:x (cl-transforms:origin vec)) adder) 
			      (cl-transforms:y (cl-transforms:origin vec)) 
			      (cl-transforms:z (cl-transforms:origin vec)))
			     (cl-transforms:make-quaternion
			      (cl-transforms:x (cl-transforms:orientation vec))
			      (cl-transforms:y (cl-transforms:orientation vec))
			      (cl-transforms:z (cl-transforms:orientation vec))
			      (cl-transforms:w (cl-transforms:orientation vec))))))))))



(defun visualize-in-y (vec offset)
  (format t "visualizing in y direction~%")
  (let*((adder 0)
	(caller 0))
    (prolog `(and (bullet-world ?w)
		  (assert (object ?w mesh sphere0 
				  ((,(cl-transforms:x (cl-transforms:origin vec)) 
				     ,(cl-transforms:y (cl-transforms:origin vec)) 
				     ,(cl-transforms:z (cl-transforms:origin vec)))
				   (,(cl-transforms:x (cl-transforms:orientation vec)) 
				     ,(cl-transforms:y (cl-transforms:orientation vec)) 
				     ,(cl-transforms:z (cl-transforms:orientation vec))
				     ,(cl-transforms:w (cl-transforms:orientation vec))))
				  :mesh cognitive-reasoning::sphere 
				  :mass 0.2 :color (0 1 0) :scale 3.0))))
    (cond ((< offset (cl-transforms:y (cl-transforms:orientation vec)))
	   (loop until (not (equal nil (prolog `(and 
						 (bullet-world ?w)
						 (contact ?w ,
							  (read-from-string 
							   (format nil "sphere~a" caller)) ?sem)))))
	      do
		(setf adder (- adder1 1))
		(setf caller (+ caller 1))
		(add-sphere (read-from-string (format nil "sphere~a" caller)) 
			    (cl-transforms:make-pose 
			     (cl-transforms:make-3d-vector 
			     (cl-transforms:x (cl-transforms:origin vec))
			     (+ (cl-transforms:y (cl-transforms:origin vec)) adder) 
			      (cl-transforms:z (cl-transforms:origin vec)))
			     (cl-transforms:make-quaternion
			      (cl-transforms:x (cl-transforms:orientation vec))
			      (cl-transforms:y (cl-transforms:orientation vec))
			      (cl-transforms:z (cl-transforms:orientation vec))
			      (cl-transforms:w (cl-transforms:orientation vec)))))))
	  ((> offset (cl-transforms:x (cl-transforms:orientation vec)))
	   (loop until (not (equal nil (prolog `(and 
						 (bullet-world ?w)
						 (contact ?w ,
							  (read-from-string 
							   (format nil "sphere~a" caller)) ?sem)))))
	      do
		(setf adder (+ adder1 1))
		(setf caller (+ caller 1))
		(add-sphere (read-from-string (format nil "sphere~a" caller)) 
			    (cl-transforms:make-pose 
			     (cl-transforms:make-3d-vector 
			      (cl-transforms:x (cl-transforms:origin vec)) 
			      (+ (cl-transforms:y (cl-transforms:origin vec)) adder) 
			      (cl-transforms:z (cl-transforms:origin vec)))
			     (cl-transforms:make-quaternion
			      (cl-transforms:x (cl-transforms:orientation vec))
			      (cl-transforms:y (cl-transforms:orientation vec))
			      (cl-transforms:z (cl-transforms:orientation vec))
			      (cl-transforms:w (cl-transforms:orientation vec))))))))))


(defun visualize-in-z (vec offset)
  (format t "visualizing in z direction~%")
  (let*((adder 0)
	(caller 0))
    (prolog `(and (bullet-world ?w)
		  (assert (object ?w mesh sphere0 
				  ((,(cl-transforms:x (cl-transforms:origin vec)) 
				     ,(cl-transforms:y (cl-transforms:origin vec)) 
				     ,(cl-transforms:z (cl-transforms:origin vec)))
				   (,(cl-transforms:x (cl-transforms:orientation vec)) 
				     ,(cl-transforms:y (cl-transforms:orientation vec)) 
				     ,(cl-transforms:z (cl-transforms:orientation vec))
				     ,(cl-transforms:w (cl-transforms:orientation vec))))
				  :mesh cognitive-reasoning::sphere 
				  :mass 0.2 :color (0 1 0) :scale 3.0))))
    (cond ((< offset (cl-transforms:z (cl-transforms:orientation vec)))
	   (loop until (not (equal nil (prolog `(and 
						 (bullet-world ?w)
						 (contact ?w ,
							  (read-from-string 
							   (format nil "sphere~a" caller)) ?sem)))))
	      do
		(setf adder (- adder1 1))
		(setf caller (+ caller 1))
		(add-sphere (read-from-string (format nil "sphere~a" caller)) 
			    (cl-transforms:make-pose 
			     (cl-transforms:make-3d-vector 
			     (cl-transforms:x (cl-transforms:origin vec))
			     (cl-transforms:y (cl-transforms:origin vec))  
			     (+ (cl-transforms:z (cl-transforms:origin vec)) adder))
			     (cl-transforms:make-quaternion
			      (cl-transforms:x (cl-transforms:orientation vec))
			      (cl-transforms:y (cl-transforms:orientation vec))
			      (cl-transforms:z (cl-transforms:orientation vec))
			      (cl-transforms:w (cl-transforms:orientation vec)))))))
	  ((> offset (cl-transforms:x (cl-transforms:orientation vec)))
	   (loop until (not (equal nil (prolog `(and 
						 (bullet-world ?w)
						 (contact ?w ,
							  (read-from-string 
							   (format nil "sphere~a" caller)) ?sem)))))
	      do
		(setf adder (+ adder1 1))
		(setf caller (+ caller 1))
		(add-sphere (read-from-string (format nil "sphere~a" caller)) 
			    (cl-transforms:make-pose 
			     (cl-transforms:make-3d-vector 
			      (cl-transforms:x (cl-transforms:origin vec)) 
			      (cl-transforms:y (cl-transforms:origin vec)) 
			      (+ (cl-transforms:z (cl-transforms:origin vec)) adder))
			     (cl-transforms:make-quaternion
			      (cl-transforms:x (cl-transforms:orientation vec))
			      (cl-transforms:y (cl-transforms:orientation vec))
			      (cl-transforms:z (cl-transforms:orientation vec))
			      (cl-transforms:w (cl-transforms:orientation vec))))))))))

(defun send-msg ()
  (let ((pub (advertise "sendMsg" "designator_integration_msgs/Designator")))
    (publish pub (desig-int::designator->msg (command-into-designator)))))


 
(defun spawn-objects ()
 (roslisp:ros-info (sherpa-spatial-relations) "SPAWN OBJECTS INTO WORLD")
 (prolog `(and (bullet-world ?w)
               	 (assert (object ?w mesh sphere5 ((-79.89979 -75.41413 29.02028)(0.498 0.0 0.0 -0.867))
				 :mesh cognitive-reasoning::sphere :mass 0.2 :color (0 0 1) :scale 3.0))))
 (prolog `(and (bullet-world ?w)
               	 (assert (object ?w mesh victim ((9 0 0)(0 0 0 1))
				 :mesh btr::victim :mass 0.2 :color (1 0 0) :scale 0.6)))))
	

(defun add-sphere (name pose)
;; position of the joint
  (let* ((origin (cl-transforms:origin pose))
         (orientation (cl-transforms:orientation pose))
         (vec-x (cl-transforms:x origin))
         (vec-y (cl-transforms:y origin))
         (vec-z (cl-transforms:z origin))
         (ori-x (cl-transforms:x orientation))
         (ori-y (cl-transforms:y orientation))
         (ori-z (cl-transforms:z orientation))
         (ori-w (cl-transforms:w orientation)))
   (format t "origin ~a~%" origin)
    (format t "what is name ~a~%" name)
    (prolog `(and (bullet-world ?w)
                  (assert (object ?w mesh ,name ((,vec-x ,vec-y ,vec-z)(,ori-x ,ori-y ,ori-z ,ori-w))
                          :mesh cognitive-reasoning::sphere :mass 0.2 :color (0 1 0) :scale 3.0))))))
;; (format t "ja endlich~%")
;;  (simple-knowledge:spawn-objects)
      

;rosrun tf static_transform_publisher 0 0 0 1.5 0 0 map world 100

;;;;;PROJECTION;;;;;

;; (cpl-impl:def-top-level-cram-function find-obj-in-world ()
;;   (cram-projection:with-projection-environment
;;       pr1qojection-process-modules::pr2-bullet-projection-environment
;;     (loop for i from 1 to 5 do
;;     (let ((obj (make-designator

;;(cpl-impl:top-level...)



(cpl-impl:def-cram-function create-loc-desig (obstacle-name)
(sb-ext:gc :full t)
(format t "creating a location designator~%")
(let ((loc (make-designator 'location `((desig-props:at ,obstacle-name)))))
  (sb-ext:gc :full t)
 loc))

(cpl-impl:def-cram-function create-obj-desig (obj-type obstacle)
(sb-ext:gc :full t)
(format t "creating an object designator~%")
(let ((object (make-designator 'desig-props:object `((desig-props:type ,obj-type)
                                           (desig-props:at ,obstacle)))))
  (sb-ext:gc :full t)
  object))

;; (cpl-impl:def-cram-function find-object-in-world (object-type obstacle-name)
;;    (cpl-impl:top-level
;;      (cram-projection:with-projection-environment
;;         projection-process-modules::pr2-bullet-projection-environment
;;       (let ((obj (put-object-from-counter-on-table type)))
;; obj)))))

;; (cpl-impl:def-cram-function find-object-in-world (object-type obstacle-name)
;;   "Return an object designator."
;;  (sbr-l--ext:gc :full t)
;;   (with-process-modules
;;   (cram-language-designator-support:with-designators
;;       ((on-obstacle (desig-props:location `(;; (desig-props:to desig-props:see)
;;                                             (pointed-pos ,(get-object-pose obstacle-name)))))
;;        ;; (close-to ,(get-object-pose (get-object-name-from-type object-type)))
;;        ;; (desig-props:obj ,(get-object-from-name (get-object-name-from-type object-type)))
       
;;        (the-object (desig-props:object `((desig-props:name ,object-type)
;;                                         (desig-props:at ,(get-object-pose obstacle-name))))))
;;     (reference on-obstacle)
;;     (format t "perceive the object: ~a~%" the-object)
;;     (let ((perceived-object (plan-lib:perceive-object 'cram-plan-library:a the-object)))
;;        (unless (desig-equal the-object perceived-object)
;;          (equate the-object perceived-object))
;;        the-object))))

;; (defun put-stuff-on-table ()
;; (sb-ext:gc :full t)
;;   (cram-projection:with-projection-environment
;;       projection-process-modules::pr2-bullet-projection-environment
;;     (cpl:top-level 
;;       (let ((desig (find-object-in-world 'human 'tree-5)))
;;         desig))))
    
;; (defun find-victim()
;;  (sb-ext:gc :full t)
;;  ;; (sb-ext:gc :full t)
;;  (cpl-impl:top-level 
;;  (cram-projection:with-projection-environment
;;      agents-projection-process-modules::agents-bullet-projection-environment
;;    (let ((object-desig (find-object-in-world  'cognitive-reasoning::victim "tree0")))
;;      (sb-ext:gc :full t)
;;      (cram-language-designator-support:with-designators
;;          ((victims-location (location `((at "Tree")
;;                                         (name "tree0")
;;                                         (for ,object-desig)
;;                                         (pointed-pos ,(get-object-pose 'sphere))))))
;;        (format t "now trying to achieve location of victim in world~%")
;;        ;; (plan-knowledge:achieve
;;        ;;  `(plan-knowledge:loc ,victims-location))
;;        ))
;;       (sb-ext:gc :full t))))
        ;; (plan-knowledge:achieve
        ;;  `(plan-knowledge:loc ,the-object ,pointed))))))
  ;;   (format t "maa is ~%")
   ;; (setf ma (plan-lib:perceive-object 'plan-lib:a the-object))
    ;; (format t "maa is ~a~%" ma)
    ;; (format t "in the new function find-object-in-world~%")
    ;; (format t "obstacle ~a~%" pointed)
    ;; (plan-lib:perceive-object 'plan-lib:a the-object)
    ;; ))

;; (defmacro with-projection-process-modules (&body body)
;;   `(cram-projection:with-projection-environment
;;        (agents-projection-process-modules::agents-bullet-projection-environment)
;;   ,@body))
(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
       (agents-navigation-process-module:agents-navigation-process-module
        ;; agents-projection-process-modules:agents-projection-process-modules
        ;; gazebo-perception-pm:gazebo-perception-process-module
        )
     ,@body))

(defun create-some-maps ()
 (let* ((transform (get-object-pose 'sphere))
        (desig (make-designator 'desig-props:location `((pointed-pos ,transform)))))
   (reference desig)))

(defun create-some-maps2 ()
  (setf desig (make-designator 'desig-props:location `((pointed-pos ,(get-object-pose 'sphere))
                                                           (desig-props:to desig-props:see)
                                                           (desig-props:object victim))))
desig)

(defun checking ()
  (let ((pose (reference (create-some-maps2))))
    (set-object-new-pose pose 'quadrotor)
  (cond ((eq nil (prolog `(and (bullet-world ?w)(robot quadrotor)
                               (visible ?w quadrotor victim))))
         (checking))
        (t
         (format t "great position~%")))))

 ;; (make-designator 'desig-props:object `((desig-props:name victim)
;;                                        (desig-props:at ,(get-object-pose 'sphere))))
;; (cpl-impl:top-level (with-process-modules 
                                         ;; (plan-lib:perceive-object 'cram-plan-library:a desig)))


(cpl-impl:def-cram-function find-object-in-world (object-type obj-name)
  "Returns an object designator"
  (cram-language-designator-support:with-designators
      ((in-world (desig-props:location `((desig-props:on "Tree")
                                         (desig-props:name ,obj-name))));;tree0 ;;cognitive-reasoning::victim
       (the-object (desig-props:object `((desig-props:type ,object-type)
                                         (desig-props:in ,in-world)))))
    ;; (reference in-world)
    (format t "trying to perceive an object ~a~%" the-object)
  (let ((perceived-object (plan-lib:perceive-object 'cram-plan-library:a the-object)))
      (unless (desig-equal the-object perceived-object)
        (equate the-object perceived-object))
      the-object)
    ))

;; (defun all()
;;   (start-world-with-robots)
;;   (spawn-objects)
;;   (format t "now spawning objects~%")
;;   (crs:prolog
;;    `(assert (btr:joint-state ?w human (("right_shoulder_joint_x" 0.06) ;;0.1
;;                                        ("right_shoulder_joint_y" -0.25)  ;;0.0 0.40
;;                                        ("right_shoulder_joint_z" 1.4)  ;;0.6 0.500
;;                                        ("left_upper_arm_joint_x" 0.1)
;;                                        ("left_upper_arm_joint_y" 3.0)
;;                                        ("left_upper_arm_joint_z" -0.5)))))
;;   (add-sphere (cl-transforms:origin (cl-transforms:make-pose  (cl-transforms:make-3d-vector 6 2 0) (cl-transforms:make-quaternion 0 0 0 1))))
;;   (marker))

(defun cheK()
 (let(( desig (make-designator 'desig-props:object `((desig-props:name victim)
                                        (desig-props:at ,(get-object-pose 'sphere))))))
   (cpl-impl:top-level (with-process-modules 
                         (plan-lib:perceive-object 'cram-plan-library:a desig)))))
;; (cpl-impl:top-level (plan-lib:perceive-object 'cram-plan-library:a desig))

;; (defun test-costmap ()
;;   (prolog `(and (costmap-padding ?pad)
;;               (costmap ?cm)
;;               (occupancy-grid-costmap::drivable-location-costmap ?cm ?pad)
;;               (semantic-map-objects ?objects)
;;               (costmap-add-function semantic-map-costmap::semantic-map-free-space
;;                                     (semantic-map-costmap::make-semantic-map-costmap
;;                                      ?objects :invert t :padding ?pad)
;;                                     ?cm)
;;               (costmap-add-function restricted-area
;;                                     (make-restricted-area-cost-function)
;;                                     ?cm)
;;               (debug-costmap ?cm))))