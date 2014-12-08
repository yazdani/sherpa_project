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

(defclass gaussian-generator () ())     
(defmethod costmap-generator-name->score ((name gaussian-generator)) 6)
(defclass range-generator () ())
(defmethod costmap-generator-name->score ((name range-generator)) 2)

(def-fact-group cognitive-reasoning-costmap (desig-costmap)
 (<- (desig-costmap ?desig ?costmap)
     (format "prolog: cognitive-reasoning-costmap~%")
   (bullet-world ?world)
   (robot ?robot)
     ; (desig-prop ?desig (right-of ?pos-loc))
   (desig-prop ?desig (pointed-pos ?pos-loc))
   ;; (desig-prop ?desig (close-to ?obj-loc)
   (robot ?robot)
   ;; (robot agents-model-description::quad)
   ;; (agents-knowledge::robot ?robot)
   (costmap ?costmap)
   ;; (costmap-add-function collisions
   ;;                       (make-costmap-bbox-gen ?objs :invert t :padding -0.2)
   ;;                       ?costmap)
   (costmap-add-function reasoning-generator
                         (make-cognitive-reasoning-cost-function ?pos-loc :Y  < 0.0)
                         ?costmap)
   (costmap ?costmap)
   (instance-of range-generator ?range-generator-id-1)
   (costmap-add-function ?range-generator-id-1
			 (make-range-cost-function ?pos-loc 3.5)
			 ?costmap)
   ;;  (costmap ?costmap)
   ;; (instance-of gaussian-generator ?gaussian-generator-id)
   ;; (format "in visibility3~%")
   ;; (costmap-add-function ?gaussian-generator-id
   ;;                       (make-location-cost-function ?pos-loc  1.0)
   ;;                       ?costmap)
   )
  
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
     (member ?type ?types))
  
  (<- (robots-checker ?world)
    (bullet-world ?world)
    (format "end checker of rob~%")
    (robot ?rob)
    (object ?world ?obj)
    (-> (== human ?rob)
        (format "hello human~%")
        (-> (== ?rob pr2) 
        (format "hello quad~%")
        (-> (== ?rob quadrotor)
            (format "ya~%")
            (format "h 2~%"))))
    (format "end checker of rob ~a~%" ?rob))
) 

  
  ;; (findall ?obj (and
   ;;                (bullet-world ?world)
   ;;                (object ?world ?name)
   ;;                (%object ?world ?name ?obj)
   ;;                (lisp-type ?obj environment-object)
   ;;                (get-slot-value ?obj types ?types)
   ;;                (member ?type ?types)) ?objs)
   ;; (format "?objs: ~a~%" (force-ll ?objs))  
   
    ;; (costmap ?costmap)
    ;; (costmap-add-function
    ;;  collisions
    ;;  (make-costmap-bbox-generator ?objs :invert t :padding -0.2)
    ;;  ?costmap)
  