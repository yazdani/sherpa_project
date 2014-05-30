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

(defmethod costmap-generator-name->score ((name (eql 'reasoning-generator))) 5)
(defmethod costmap-generator-name->score ((name (eql 'collisions))) 10)

(def-fact-group cognitive-reasoning-costmap (desig-costmap)
 (<- (desig-costmap ?desig ?costmap)
    (bullet-world ?world)
   (format "helooo~%")
 ;; (findall ?obj (and
    ;;                (bullet-world ?world)
    ;;                (object ?world ?name)
    ;;                (%object ?world ?name ?obj)
    ;;                (lisp-type ?obj environment-object)
    ;;                (get-slot-value ?obj types ?types)
    ;;                (member ?type ?types)) ?objs)
    ;; (format "?objs: ~a~%" ?objs)
    ;; (costmap ?costmap)
    ;; (costmap-add-function
    ;;  collisions
    ;;  (make-costmap-bbox-generator ?objs :invert t :padding -0.2)
    ;;  ?costmap)
   (desig-prop ?desig (right-of ?pos-tree))
    (format "?objs: ~%")
    ;; (agents-knowledge::robot ?robot)
    (format "robot ~a~%")
    (costmap ?costmap)
    (format "?oAAAbjs: ~%")
    (costmap-add-function reasoning-generator
                          (make-cognitive-reasoning-cost-function ?pos-tree :Y  < 0.0)
                          ?costmap))
  (<- (environment-object-type ?world ?name ?type)
    (bullet-world ?world)
    (object ?world ?name)
    (%object ?world ?name ?object-instance)
    (lisp-type ?object-instance environment-object)
    (get-slot-value ?object-instance types ?types)
    (member ?type ?types))

 (<- (human-specific-object-type ?world ?name ?type)
    (bullet-world ?world)
    (object ?world ?name)
    (%object ?world ?name ?object-instance)
    (lisp-type ?object-instance human-specific-object)
    (get-slot-value ?object-instance types ?types)
    (member ?type ?types)))