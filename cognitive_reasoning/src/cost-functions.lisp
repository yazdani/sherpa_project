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

(defun make-cognitive-reasoning-cost-function (location axis pred threshold)
  (roslisp:ros-info (sherpa-spatial-relations) "calculate the costmap")
  (let* ((transformation (cl-transforms:pose->transform location))
         (world->location-transformation (cl-transforms:transform-inv transformation)))
    (roslisp:ros-info (sherpa-spatial-relations) "hi")
    (format t "location is: ~a~%" location)
    (lambda (x y)
      (let* ((point (cl-transforms:transform-point world->location-transformation
                                                   (cl-transforms:make-3d-vector x y 0)))
             (coord (ecase axis
                      (:x (cl-transforms:x point))
                      (:y (cl-transforms:y point))))
             (mode (sqrt (+  (* (cl-transforms:x point) (cl-transforms:x point))
                             (* (cl-transforms:y point) (cl-transforms:y point))))))
        (if (funcall pred coord 0.0d0)
            (if (> (abs (/ coord mode)) threshold)
                (abs (/ coord mode))
                0.0d0)
            0.0d0)))))

(defun make-costmap-bbox-gen (objs &key invert padding)
(force-ll objs)
  (when objs
    (let ((aabbs (loop for obj in (cut:force-ll objs)
                       collecting (btr:aabb obj))))
      (format t "aabbs: ~a~%" aabbs)
      (lambda (x y)
        (block nil
          (dolist (bounding-box aabbs (if invert 1.0d0 0.0d0))
            (let* ((bb-center (cl-bullet:bounding-box-center bounding-box))
                   (dimensions-x/2
                     (+ (/ (cl-transforms:x (bt:bounding-box-dimensions bounding-box)) 2)
                        padding))
                   (dimensions-y/2
                     (+ (/ (cl-transforms:y (bt:bounding-box-dimensions bounding-box)) 2)
                        padding)))

              (when (and
                     (< x (+ (cl-transforms:x bb-center) dimensions-x/2))
                     (> x (- (cl-transforms:x bb-center) dimensions-x/2))
                     (< y (+ (cl-transforms:y bb-center) dimensions-y/2))
                     (> y (- (cl-transforms:y bb-center) dimensions-y/2)))
                ;; (format t "here2~%")
                (return (if invert 0.0d0 1.0d0)))
              )))))) )

;; (defun make-costmap-bbox-gen-object (object)
;;   (let* ((bounding-box (btr::aabb object))
;;          (dimensions-x/2 (/ (cl-transforms:x (bt:bounding-box-dimensions bounding-box))
;;                             2))
;;          (dimensions-y/2 (/ (cl-transforms:y (bt:bounding-box-dimensions bounding-box))
;;                             2)))
;;     (lambda (x y)
;;       (if (and
;;            (< x (+ (cl-transforms:x (cl-bullet:bounding-box-center bounding-box))
;;                    dimensions-x/2))
;;            (> x (- (cl-transforms:x (cl-bullet:bounding-box-center bounding-box))
;;                    dimensions-x/2))
;;            (< y (+ (cl-transforms:y (cl-bullet:bounding-box-center bounding-box))
;;                    dimensions-y/2))
;;            (> y (- (cl-transforms:y (cl-bullet:bounding-box-center bounding-box))
;;                    dimensions-y/2)))
;;        1.0d0))))
        

;; (defun make-constant-height-function (height)
;;   (format t "list height123: ~a~%" (list height))
;;   (lambda (x y)
;;     (declare (ignore x y))
;;     (list height)))



(defun make-constant-height-function (height)
  (format t "list height123: ~a~%" (list height))
  (let* ((lists (force-ll
		 (prolog `(and (bullet-world ?w)
                               (human-object-type ?w ?name ?type)
			       (pose ?w ?name ?pose)))))
	 (pose (cdr (assoc '?pose (car lists))))
	 (z-origin (cl-transforms:z (cl-transforms:origin pose)))
	 (rob-name (cdadar (force-ll
			    (prolog `(and (bullet-world ?w)
					  (robot ?robot)))))))
    (format t "rob-name is ~a~%" rob-name)
    (cond ((string-equal rob-name 'quadrotor)
	   (format t "wir sind im ersten Teil von height und addieren die Zwei~%")
	   (setf height (+ 2 z-origin)))
	  (t (format t "sind anscheinend noch bei zwei~%")))
    (lambda (x y)
      (declare (ignore x y))
      (list height))))
	 
