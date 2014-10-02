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


;; (defparameter *cone-pose* (cl-transforms:make-pose
;;                            (cl-transforms:make-3d-vector 6.9 -0.22 3.85)
;;                            (cl-transforms:axis-angle->quaternion
;;                             (cl-transforms:make-3d-vector 0 1 0) 
;;                             -1.85)))

(defun start-myros ()
  (roslisp:ros-info (sherpa-spatial-relations) "START the ROSNODE")
  (roslisp-utilities:startup-ros :anonymous nil))
(defun end-myros ()
  (roslisp:ros-info (sherpa-spatial-relations) "KILL the ROSNODE")
  (roslisp-utilities:shutdown-ros))

(defun start-world-with-robots ()
  (roslisp:ros-info (sherpa-spatial-relations) "START WORLD AND ROBOTS")
  (location-costmap::location-costmap-vis-init)
  (setf *list* nil)
  (let* ((genius-urdf (cl-urdf:parse-urdf (roslisp:get-param "human/robot_description")))
         (quad-urdf (cl-urdf:parse-urdf (roslisp:get-param "quadrotor/robot_description")))
         ;; (rover-urdf (cl-urdf:parse-urdf (roslisp:get-param "rover/robot_description")))
         )
    (setf *list*
          (car 
           (btr::force-ll
            (btr::prolog
             `(and
	       (clear-bullet-world)
               (bullet-world ?w)
               (assert (object ?w static-plane floor ((0 0 0) (0 0 0 1))
                               :normal (0 0 1) :constant 0 :no-robot-collision t))
	       (debug-window ?w)
	       (assert (object ?w urdf human ((0 0 0) (0 0 1 1)) :urdf ,genius-urdf))
	       (assert (object ?w urdf quadrotor ((-1 -2 2)(0 0 1 1)) :urdf ,quad-urdf))
 	       ;; (assert (object ?w urdf quadrotor ((-1 -2 0.2)(0 0 1 1)) :urdf ,quad-urdf))
 
	       ;; (assert (object ?w urdf rover ((1 3 0) (0 0 0 1)) :urdf ,rover-urdf))
         )))))))
 
(defun spawn-objects ()
 (roslisp:ros-info (sherpa-spatial-relations) "SPAWN OBJECTS INTO WORLD")
 (prolog `(and (bullet-world ?w)
		 (assert (object ?w mesh tree-5 ((6 1 0)(0 0 0 1))
				 :mesh cognitive-reasoning::tanne1 :mass 0.2 :color (0 0.5 0)))
		 (assert (object ?w mesh tree-6 ((10 4 0)(0 0 0 1))
				 :mesh cognitive-reasoning::tanne1 :mass 0.2 :color (0 0 0)))
		 (assert (object ?w mesh tree-7 ((10 -4 0)(0 0 0 1))
				 :mesh cognitive-reasoning::tanne1 :mass 0.2 :color (0 0 0)))
		 (assert (object ?w mesh tree-8 ((15 2 0)(0 0 0 1))
				 :mesh cognitive-reasoning::tanne2 :mass 0.2 :color (0 0.5 0))) 
		 (assert (object ?w mesh tree-9 ((13 -6 0)(0 0 0 1))
				 :mesh cognitive-reasoning::tanne2 :mass 0.2 :color (0 0.5 0))) 
		 (assert (object ?w mesh tree-10 ((10 -8 0)(0 0 0 1))
				 :mesh cognitive-reasoning::tanne1 :mass 0.2 :color (0 0 0)))
		 (assert (object ?w mesh tree-12 ((4 -8 0)(0 0 0 1))
				 :mesh cognitive-reasoning::tanne1 :mass 0.2 :color (0 0.5 0)))
		 (assert (object ?w mesh victim ((9 0 0)(0 0 0 1))
				 :mesh cognitive-reasoning::victim :mass 0.2 :color (1 0 0) :scale 0.6))))
  (simple-knowledge::clear-object-list)
   (simple-knowledge::add-object-to-spawn
   :name "tree-5"
   :type 'tree
   :collision-parts nil
   :pose (tf:make-pose-stamped
          "/map"
          0.0
          (tf:make-3d-vector 6 1 0)
          (tf:make-quaternion 0 0 0 1))
   :file (model-path "tanne1.urdf"))
  (simple-knowledge::add-object-to-spawn
   :name "tree-6"
   :type 'tree
   :collision-parts nil
   :pose (tf:make-pose-stamped
          "/map"
          0.0
          (tf:make-3d-vector 10 4 0)
          (tf:make-quaternion 0 0 0 1))
   :file (model-path "tanne12.urdf"))
  (simple-knowledge::add-object-to-spawn
   :name "tree-7"
   :type 'tree
   :collision-parts nil
   :pose (tf:make-pose-stamped
          "/map"
          0.0
          (tf:make-3d-vector 10 -4 0)
          (tf:make-quaternion 0 0 0 1))
   :file (model-path "tanne12.urdf"))
 (simple-knowledge::add-object-to-spawn
   :name "tree-8"
   :type 'tree
   :collision-parts nil
   :pose (tf:make-pose-stamped
          "/map"
          0.0
          (tf:make-3d-vector 15 2 0)
          (tf:make-quaternion 0 0 0 1))
   :file (model-path "tanne2.urdf"))
 (simple-knowledge::add-object-to-spawn
   :name "tree-9"
   :type 'tree
   :collision-parts nil
   :pose (tf:make-pose-stamped
          "/map"
          0.0
          (tf:make-3d-vector 13 -6 0)
          (tf:make-quaternion 0 0 0 1))
   :file (model-path "tanne21.urdf"))
 (simple-knowledge::add-object-to-spawn
   :name "tree-10"
   :type 'tree
   :collision-parts nil
   :pose (tf:make-pose-stamped
          "/map"
          0.0
          (tf:make-3d-vector 10 -8 0)
          (tf:make-quaternion 0 0 0 1))
   :file (model-path "tanne12.urdf"))
 (simple-knowledge::add-object-to-spawn
   :name "tree-12"
   :type 'tree
   :collision-parts nil
   :pose (tf:make-pose-stamped
          "/map"
          0.0
          (tf:make-3d-vector 4 -8 0)
          (tf:make-quaternion 0 0 0 1))
   :file (model-path "tanne1.urdf"))
 (simple-knowledge::add-object-to-spawn
   :name "victim"
   :type 'clothes
   :collision-parts nil
   :pose (tf:make-pose-stamped
          "/map"
          0.0
          (tf:make-3d-vector 9 0 0)
          (tf:make-quaternion 0 0 0 1))
   :file (model-path "victim.urdf"))
   (simple-knowledge:spawn-objects)
  )

(defun tester()
  (let ((pose (pointed-direction))
        )
    (format t "hello sweety~%")
    (add-sphere pose)
    (check-collision)))
        
(defun check-collision ()
 ;(add-sphere pose)
(let* ((pose (get-object-pose 'sphere3))
       (collision-detector (prolog `(and
                                     (bullet-world ?w)
                                     (contact ?w sphere3 ?objs))))
       (vector (cl-transforms:origin pose))
       (vec-y (cl-transforms:y vector))
       (vec-x  (cl-transforms:x vector))
       (vec-z   (cl-transforms:z vector))
       (new-vec-y (+ vec-y -1)))
  (cond ((eq nil collision-detector)(format t "great job, take this one ~%"))
        (t
         (prolog `(and 
                   (bullet-world ?w)
                   (assert
                    (object-pose ?w sphere3 ((,vec-x ,new-vec-y ,vec-z) (0 0 0 1))))))
         (format t "hellooooo~%")))))

(defun pointed-direction ()
 ;; started rosrun nodes
  (location-costmap::location-costmap-vis-init)
  (let* ((transform-x (tf:lookup-transform cram-roslisp-common:*tf* :time 0.0 :source-frame "right_hand_x" :target-frame "map"))
         (trans-x (cl-transforms:transform->pose transform-x))
         (trans (cl-transforms:origin trans-x))
         (x-val (+ (cl-transforms:x trans) 5))
         (new-vec (cl-transforms:make-3d-vector x-val (cl-transforms:y trans) (cl-transforms:z trans))))
         (publish-point new-vec)
    (format t "the end ~a~%" new-vec)
    new-vec))

(defun add-sphere2 ()
;; position of the joint
 (prolog `(and (bullet-world ?w)
               (assert (object ?w mesh sphere3 ((6 1 0)(0 0 0 1))
                                  :mesh cognitive-reasoning::sphere :mass 0.2 :color (0 0.5 0))))))


(defun add-sphere (pose)
;; position of the joint
  (format t "pose is: ~a~%" pose)
  (let* (;(vector (cl-transforms:origin pose))
         (vec-x (cl-transforms:x pose))
        (vec-y (cl-transforms:y pose))
        (vec-z (cl-transforms:z pose)))
    (format t "ich bin nun hier ~%")
      (prolog `(and (bullet-world ?w)
                  (assert (object ?w mesh sphere ((,vec-x ,vec-y ,vec-z)(0 0 0 1))
                                  :mesh cognitive-reasoning::sphere :mass 0.2 :color (0 0.5 0)))))
    (simple-knowledge::clear-object-list)
    (simple-knowledge::add-object-to-spawn
     :name "sphere15"
     :type 'collision-detector
     :collision-parts nil
     :pose 
     (tf:make-pose-stamped
      "/map"
      0.0
      (tf:make-3d-vector vec-x vec-y vec-z)
          (tf:make-quaternion 0 0 0 1))
   :file (model-path "sphere.urdf"))
  (format t "ja endlich~%")
   (simple-knowledge:spawn-objects)
  ))

;rosrun tf static_transform_publisher 0 0 0 1.5 0 0 map odom_combined 100


(defun model-path (name)
  (physics-utils:parse-uri
   (concatenate
    'string
    "package://world_model_description/urdf/"
    name)))

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

(cpl-impl:def-cram-function find-object-in-world (object-type obstacle-name)
  "Return an object designator."
 (sb-ext:gc :full t)
  (cram-language-designator-support:with-designators
      ((on-obstacle (desig-props:location `(;; (desig-props:to desig-props:see)
                                            (right-of ,(get-object-pose obstacle-name)))))
       ;; (close-to ,(get-object-pose (get-object-name-from-type object-type)))
       ;; (desig-props:obj ,(get-object-from-name (get-object-name-from-type object-type)))
       
       (the-object (desig-props:object `((desig-props:name ,object-type)
                                        (desig-props:at ,(get-object-pose obstacle-name))))))
    (reference on-obstacle)
    (format t "perceive the object: ~a~%" the-object)
    (let ((perceived-object (plan-lib:perceive-object 'cram-plan-library:a the-object)))
       (unless (desig-equal the-object perceived-object)
         (equate the-object perceived-object))
       the-object)))

;; (defun put-stuff-on-table ()
;; (sb-ext:gc :full t)
;;   (cram-projection:with-projection-environment
;;       projection-process-modules::pr2-bullet-projection-environment
;;     (cpl:top-level 
;;       (let ((desig (find-object-in-world 'human 'tree-5)))
;;         desig))))
    
;;   ;; (cram-projection:with-projection-environment
;;   ;;     projection-process-modules::pr2-bullet-projection-environment
;;       (cram-language-designator-support:with-designators
;;        ((on-obstacle (desig-props:location `(;; (desig-props:to desig-props:see)
;;                                           (right-of ,(get-object-pose obstacle-name)))))
;;                                              ;; (close-to ,(get-object-pose (get-object-name-from-type object-type)))
;;                                                    ;; (desig-props:obj ,(get-object-from-name (get-object-name-from-type object-type)))
                                           
;;        (the-object (desig-props:object `((desig-props:type ,object-type)
;;                                          (desig-props:at ,on-obstacle)))))
;;              (reference on-obstacle)
;;     (format t "maa is ~%")
;;     ;; (setf ma (plan-lib:perceive-object 'plan-lib:a the-object))
;;     ;; (format t "maa is ~a~%" ma)
;;     (format t "in the new function find-object-in-world~%")
;;     (format t "obstacle ~a~%" on-obstacle)
;;     ;; (reference on-obstacle)
;;     ;; (plan-lib:perceive-object 'plan-lib:a the-object)
;;     ))

;; (defmacro with-process-modules (&body body)
;;   `(cpm:with-process-modules-running
;;        (pr2-projection-process-modules:projection-process-modules)
;;         ;; gazebo-perception-pm:gazebo-perception-process-module)
;;      ,@body))