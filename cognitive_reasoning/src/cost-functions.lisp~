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


(defun make-object-bounding-box-costmap-gen (object)
  (let* ((bounding-box (aabb object))
         (dimensions-x/2 (/ (cl-transforms:x (bt:bounding-box-dimensions bounding-box))
                            2))
         (dimensions-y/2 (/ (cl-transforms:y (bt:bounding-box-dimensions bounding-box))
                            2)))
    (lambda (x y)
      (if (and
           (< x (+ (cl-transforms:x (cl-bullet:bounding-box-center bounding-box))
                   dimensions-x/2))
           (> x (- (cl-transforms:x (cl-bullet:bounding-box-center bounding-box))
                   dimensions-x/2))
           (< y (+ (cl-transforms:y (cl-bullet:bounding-box-center bounding-box))
                   dimensions-y/2))
           (> y (- (cl-transforms:y (cl-bullet:bounding-box-center bounding-box))
                   dimensions-y/2)))
          1.0 0.0))))

(defun make-constant-height-function (height)
  (format t "list height: ~a~%" (list height))
  (lambda (x y)
    (declare (ignore x y))
    (list height)))


