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

(defun model-path (name)
  (physics-utils:parse-uri
   (concatenate
    'string
    "package://world_model_description/urdf/"
    name)))

(defun fill-knowledge-list (name)
 (roslisp:ros-info (sherpa-spatial-relations) "Start filling knowledge list")
 (simple-knowledge::clear-object-list)
 (cond ((eq name
 (simple-knowledge::add-object-to-spawn
  :name "base-camp"
  :type 'base-camp
  :collision-parts nil
  :pose (tf:make-pose-stamped
	 "/map"
	 0.0
	 (tf:make-3d-vector -2 -2 -1)
	 (tf:make-quaternion 0 0 0 1))
  :file (model-path "sherpa_base_camp.urdf"))
 ;;  (simple-knowledge::add-object-to-spawn
 ;;   :name "tree-3"
 ;;   :type 'tree
 ;;   :collision-parts nil
 ;;   :pose (tf:make-pose-stamped
 ;;          "/map"
 ;;          0.0
 ;;          (tf:make-3d-vector 5 2 0)
 ;;          (tf:make-quaternion 0 0 0 1))
 ;;   :file (model-path "tree-3.urdf"))
 ;;  (simple-knowledge::add-object-to-spawn
 ;;   :name "tree-2"
 ;;   :type 'tree
 ;;   :collision-parts nil
 ;;   :pose (tf:make-pose-stamped
 ;;          "/map"
 ;;          0.0
 ;;          (tf:make-3d-vector 6 -1 0)
 ;;          (tf:make-quaternion 0 0 0 1))
 ;;   :file (model-path "tree-2.urdf"))
 ;;  (simple-knowledge::add-object-to-spawn
 ;;   :name "tree-1"
 ;;   :type 'tree
 ;;   :collision-parts nil
 ;;   :pose (tf:make-pose-stamped
 ;;          "/map"
 ;;          0.0
 ;;          (tf:make-3d-vector 5.5 0 0)
 ;;          (tf:make-quaternion 0 0 0 1))
 ;;   :file (model-path "tree-1.urdf"))
 ;; (simple-knowledge::add-object-to-spawn
 ;;   :name "tree-4"
 ;;   :type 'tree
 ;;   :collision-parts nil
 ;;   :pose (tf:make-pose-stamped
 ;;          "/map"
 ;;          0.0
 ;;          (tf:make-3d-vector 6 0 0)
 ;;          (tf:make-quaternion 0 0 0 1))
 ;;   :file (model-path "tree-4.urdf"))
  ;; (simple-knowledge::add-object-to-spawn
  ;;  :name "hat"
  ;;  :type 'clothes
  ;;  :collision-parts nil
  ;;  :pose (tf:make-pose-stamped
  ;;         "/map"
  ;;         0.0
  ;;         (tf:make-3d-vector 7 0 0)
  ;;         (tf:make-quaternion 0 0 0 1))
  ;;  :file (model-path "hat.urdf"))
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
 ;; (simple-knowledge::add-object-to-spawn
 ;;   :name "sherpa_rover"
 ;;   :type 'robot
 ;;   :collision-parts nil
 ;;   :pose (tf:make-pose-stamped
 ;;          "/map"
 ;;          0.0
 ;;          (tf:make-3d-vector 1 3 0)
 ;;          (tf:make-quaternion 0 0 0 1))
 ;;   :file (model-path "sherpa_rover.urdf"))
  ;; (simple-knowledge::add-object-to-spawn
  ;;  :name "sherpa_quad"
  ;;  :type 'robot
  ;;  :collision-parts nil
  ;;  :pose (tf:make-pose-stamped
  ;;         "/map"
  ;;         0.0
  ;;         (tf:make-3d-vector  -2 3.5 3)
  ;;         (tf:make-quaternion 0 0 0 1))
  ;;  :file (model-path "quadrotor.urdf"))
  )

(defun check-sof()
  (roslisp:ros-info (sherpa-spatial-relations) "Start filling knowledge list with subject of interest")
 (simple-knowledge::clear-object-list)
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

