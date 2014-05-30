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

(defun find-object (designator)
  "Finds objects with (optional) name `object-name' and type `type'
  and returns a list of elements of the form \(name pose\)."
  (let ((object-name (or
                      (when (slot-value designator 'desig:data)
                        (desig:object-identifier (desig:reference designator)))
                      (desig:desig-prop-value designator 'desig-props:name)))
        (type (or (desig:desig-prop-value designator 'desig-props:type)
                  '?_)))
    (flet ((find-human-specific-object ()
             (cut:force-ll
              (cut:lazy-mapcar
               (lambda (solution)
                 (cut:with-vars-strictly-bound (?object ?pose) solution
                   (list ?object ?pose)))
               (crs:prolog `(and (robot quad)
                                 (bullet-world ?world)
                                 ,@(when object-name
                                     `((crs:== ?object ,object-name)))
                                 (object ?world ?object)
                                 (human-specific-object-type ?world ?object ,type)
                                 (visible ?world quad ?object)
                                 (pose ?world ?object ?pose))))))
           ;; (find-handle ()
           ;;   (mapcar (lambda (semantic-map-object)
           ;;             (list
           ;;              (sem-map-utils:name semantic-map-object)
           ;;              (sem-map-utils:pose semantic-map-object)))
           ;;           (sem-map-utils:designator->semantic-map-objects designator)))
	   )
      ;; (case type
      ;;   (desig-props:handle (find-handle))
      ;;   (t 
	 (find-human-specific-object))))