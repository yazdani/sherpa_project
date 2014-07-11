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

(in-package :cognitive-reasoning)

(defparameter *mesh-files* '((tree1 "package://sherpa_spatial_relations/meshes/tree-1.stl" nil)
                             (tree2 "package://sherpa_spatial_relations/meshes/tree-2.stl" nil)
                             (tree3 "package://sherpa_spatial_relations/meshes/tree-3.stl" nil)
                             (tree4 "package://sherpa_spatial_relations/meshes/tree-4.stl" nil)
                             (tanne1 "package://sherpa_spatial_relations/meshes/tanne1.stl" nil)
                             (tanne2 "package://sherpa_spatial_relations/meshes/tanne2.stl" nil)
                             (Tent1 "package://sherpa_spatial_relations/meshes/Tent1.stl" nil)
                             (Tent2 "package://sherpa_spatial_relations/meshes/Tent2.stl" nil)
                             (hat "package://sherpa_spatial_relations/meshes/hat.stl" nil)
                             (victim "package://sherpa_spatial_relations/meshes/victim.stl" nil)
                             (mountain "package://sherpa_spatial_relations/meshes/mountain_gen.stl" nil)
                             (plane "package://sherpa_spatial_relations/meshes/map.stl" nil)
                             (cone "package://sherpa_spatial_relations/meshes/cone.stl" nil)))


                          ;;   (hat "package://sherpa_spatial_relations/meshes/hat.stl" nil)

 (defclass environment-object (object)
   ((types :reader environment-object-types :initarg :types)))

 (defclass human-specific-object (object)
   ((types :reader human-specific-object-types :initarg :types)))

(defclass objectshape-object (object)
   ((types :reader objectshape-object-types :initarg :types)))

;;(defgeneric environment-object-dimensions (object)
;;  (:method ((object environment-object))
;;    (cl-bullet:bounding-box-dimensions (aabb object)))
;;  (:method ((object-type symbol))
;;    (or (cutlery-dimensions object-type)
;;        (let ((mesh-specification (assoc object-type *mesh-files*)))
;;          (format t "object-type ~a~%" object-type)
;;          (assert
;;           mesh-specification ()
;;           "Couldn't fine a mesh for object type ~a." object-type)
;;          (destructuring-bind (type uri &optional flip-winding-order)
;;              mesh-specification
;;            (declare (ignore type))
;;            (let ((model-filename (physics-utils:parse-uri uri)))
;;              (with-file-cache
;;                  model model-filename (physics-utils:load-3d-model
;;                                        model-filename
;;                                        :flip-winding-order flip-winding-order)
;;                (values
;;                 (physics-utils:calculate-aabb
;;                  (physics-utils:3d-model-vertices model))))))))))


(defun make-environment-object (world name types &optional bodies (add-to-world t))
  (make-instance 'environment-object
    :name name
    :world world
    :rigid-bodies bodies
    :add add-to-world
    :types types))

(defun make-objectshape-object (world name types &optional bodies (add-to-world t))
  (make-instance 'objectshape-object
    :name name
    :world world
    :rigid-bodies bodies
    :add add-to-world
    :types types))

(defun make-human-specific-object (world name types &optional bodies (add-to-world t))
 (make-instance 'human-specific-object
    :name name
    :world world
    :rigid-bodies bodies
    :add add-to-world
    :types types)
 )


(defmethod add-object ((world cl-bullet:bt-world) (type (eql 'mesh)) name pose
                       &key mass mesh (color '(0.5 0.5 0.5 1.0)) types (scale 1.0)
                         disable-face-culling)
  (format t "HALLLOOOOO ENVIRONMENT~%")
  (let ((mesh-model (physics-utils:scale-3d-model
                     (etypecase mesh
                       (symbol (let ((uri (physics-utils:parse-uri (cadr (assoc mesh *mesh-files*)))))
                                 (with-file-cache model uri                                  
                                     (physics-utils:load-3d-model
                                      uri :flip-winding-order (caddr (assoc mesh *mesh-files*)))
                                   model)))
                       (string (let ((uri  (physics-utils:parse-uri mesh)))
                                 (with-file-cache model uri (physics-utils:load-3d-model uri)
                                   model)))
                       (physics-utils:3d-model mesh))
                     scale)))
 ;;ToDO change the hard-cording stuff and string-equal (there exist different types)
    (cond  ((string-equal name 'victim)
            (make-human-specific-object world name
                                        (or types (list mesh))
                                        (list
                                         (make-instance 'rigid-body
                                                        :name name :mass mass :pose (ensure-pose pose)
                                                        :collision-shape (make-instance 'cl-bullet-vis:convex-hull-mesh-shape
                                                                                        :points (physics-utils:3d-model-vertices mesh-model)
                                                                                        :faces (physics-utils:3d-model-faces mesh-model)
                                                                                        :color color
                                                                                        :disable-face-culling disable-face-culling))))
            (format t "bitteeee~%"))
          (t (make-environment-object world name (or types (list mesh))
                                      (list
                                       (make-instance 'rigid-body
                                                      :name name :mass mass :pose (ensure-pose pose)
                                                      :collision-shape (make-instance 'cl-bullet-vis:convex-hull-mesh-shape
                                                                                      :points (physics-utils:3d-model-vertices mesh-model)
                                                                                      :faces (physics-utils:3d-model-faces mesh-model)
                                                                                      :color color
                                                                                      :disable-face-culling disable-face-culling))))))))
