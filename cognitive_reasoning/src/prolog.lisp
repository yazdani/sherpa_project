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
    (bullet-world ?world)
    (robot ?robot)
    (desig-prop ?desig (right-of ?pos-loc))
    (desig-prop ?desig (look-for ?_))
    (costmap ?costmap)
    (format "kuejm~%")
    (costmap-add-function reasoning-generator
                          (make-cognitive-reasoning-cost-function ?pos-loc :Y  < 0.0)
                          ?costmap)
    (costmap ?costmap)
    (instance-of range-generator ?range-generator-id-1)
    (format "jjsj~%")
    (costmap-add-function ?range-generator-id-1
                          (make-range-cost-function ?pos-loc 3.5)
                          ?costmap)
   )
    

  (<- (desig-costmap ?desig ?costmap)
    (format "jj~%")
    (bullet-world ?world)
    (robot ?robot)
     (format "jjdfd~%")
    (desig-prop ?desig (pointed-pos ?pos-name))
    (desig-prop ?desig (right-of ?pos-loc))
     (format "jjisos ~a~%" ?pos-name)
    (desig-location-prop ?pos-name ?pos)
     (format "jäsüsj ~a~%" ?pos)
    (desig-prop ?desig (for ?obj-name))
    (format "jjww~%")
    (findall ?obj (and
                   (bullet-world ?world)
                   (object ?world ?name)
                   (%object ?world ?name ?obj)
                   (lisp-type ?obj environment-object)
                   (get-slot-value ?obj types ?types)
                   (member ?type ?types)) ?objs)
      (costmap ?costmap)
     (costmap-add-function reasoning-generator
                          (make-cognitive-reasoning-cost-function ?pos-loc :Y  < 0.0)
                          ?costmap)
  
    (costmap ?costmap)
     (costmap-add-function collisions
                          (make-costmap-bbox-gen ?objs :invert t :padding 0.1)
                          ?costmap)
    (costmap ?costmap)
    (desig-location-prop ?obj-name ?ref-obj-pose)
    (format "jjddfffffffd~%")
    ;; (costmap-add-function reasoning-generator
    ;;                       (make-cognitive-reasoning-cost-function ?pos :Y  < 0.0)
    ;;                       ?costmap)
    ;; (costmap ?costmap)
    ;; (costmap-add-function reasoning-generator
    ;;                       (make-cognitive-reasoning-cost-function ?pos :Y  > 0.0)
    ;;                       ?costmap)
    ;; (costmap ?costmap)
    (format "jjddfddddfd~%")
    (instance-of range-generator ?range-generator-id-1)
      (format "jjddddddddd~%")
    (costmap-add-function ?range-generator-id-1
                          (make-range-cost-function ?ref-obj-pose 2.5)
                          ?costmap)
    (costmap ?costmap)
    ;; (instance-of range-generator ?range-generator-id-2)
    ;; (costmap-add-function
    ;;  ?range-generator-id-2
    ;;  (make-range-cost-function ?ref-obj-pose 1.5d0 :invert t )
    ;;  ?costmap)
    ;; (costmap ?costmap)
   (instance-of range-generator ?range-generator-id-1)
      (costmap-add-function
       ?range-generator-id-1
       (make-range-cost-function ?ref-obj-pose 1)
       ?costmap)
    (costmap ?costmap)
    (format "jjdddf~%")
    (instance-of gaussian-generator ?gaussian-generator-id)
    (costmap-add-function ?gaussian-generator-id
                          (make-location-cost-function ?ref-obj-pose  1.0)
                          ?costmap))

  
  (<- (desig-costmap ?desig ?costmap)
    (bullet-world ?world)
    (robot ?robot)
    (desig-prop ?desig (near ?ref-obj-name))
    (desig-prop ?desig (for ?for-obj-name))
    (desig-location-prop ?ref-obj-name ?ref-obj-pose)
    (format "~a~%" ?ref-obj-pose)
    (complete-object-size ?w ?ref-obj-name ?ref-obj-size)
    (complete-object-size ?w ?for-obj-name ?for-obj-size)
    (costmap ?costmap)
    (near-costmap ?desig ?ref-obj-pose ?ref-obj-size 0.1d0 ?for-obj-size 0.1d0 ?costmap))

  (<- (near-costmap ?desig ?ref-obj-pose ?ref-obj-size ?ref-padding ?for-obj-size ?for-padding ?costmap)
      (costmap ?costmap)
      (instance-of gaussian-generator ?gaussian-generator-id)
      (costmap-add-function
       ?gaussian-generator-id
       (make-location-cost-function ?ref-obj-pose 1.0d0)
       ?costmap)
      (lisp-fun calculate-near-costmap ?ref-obj-size ?for-obj-size
		?ref-padding ?for-padding ?min-radius)
      (lisp-fun calculate-costmap-width ?ref-obj-size ?for-obj-size 1.0d0
		?cm-width)
      (lisp-fun + ?min-radius ?cm-width ?max-radius)
    (format "?max-radius : ~a~%" ?max-radius)
      (instance-of range-generator ?range-generator-id-1)
      (costmap-add-function
       ?range-generator-id-1
       (make-range-cost-function ?ref-obj-pose ?max-radius)
       ?costmap)
    (format "min-radous ~a~%" ?min-radius)
       (instance-of range-generator ?range-generator-id-2)
      (costmap-add-function
       ?range-generator-id-2
       (make-range-cost-function ?ref-obj-pose ?min-radius :invert t)
       ?costmap)
    )

    
   
 

  (<- (complete-object-size ?w ?obj-name ?obj-size)
    (format "complete3~%")
    (%object ?w ?obj-name ?obj)
    (format "complete1~%")
    (%complete-obj-size ?w ?obj ?obj-size))
  
  (<- (%complete-obj-size ?w ?obj ?size)
    (format "complete2 ~a ~a~%" ?obj ?size)
  (lisp-fun get-aabb ?obj ?size))
  
  
  (<- (environment-object-type ?world ?name ?type)
    (bullet-world ?world)
    (object ?world ?name)
    (%object ?world ?name ?object-instance)
    (lisp-type ?object-instance environment-object)
    (get-slot-value ?object-instance types ?types)
    (member ?type ?types))

  ;; (<- (desig-costmap ?desig ?costmap)
  ;;   (bullet-world ?world)
  ;;   (robot ?robot)
  ;;   (desig-prop ?desig (behind ?ref-obj-name))
  ;;   (desig-prop ?desig (look-for ?for-obj-name))
  ;;   (desig-location-prop ?ref-obj-name ?ref-obj-pose)
  ;;   (format "~a~%" ?ref-obj-pose)
  ;;   (complete-object-size ?w ?ref-obj-name ?ref-obj-size)
  ;;   (complete-object-size ?w ?for-obj-name ?for-obj-size)
  ;;   (costmap ?costmap)
  ;;   (near-costmap ?desig ?ref-obj-pose ?ref-obj-size 0.1d0 ?for-obj-size 0.1d0 ?costmap))
	      
)

(def-fact-group location-desig-utils ()

  (<- (object-instance-name ?name ?name)
      (format "object-instance-name ~a~%" ?name)
      (format "      (lisp-type ?name symbol) ~a~%" (lisp-type ?name symbol))
      (lisp-type ?name symbol))
)
 ;; (<- (desig-costmap ?desig ?costmap)
  ;;   (bullet-world ?world)
  ;;   (robot ?robot)
  ;;   (desig-prop ?desig (pointed-pos ?pos))
  ;;   (desig-prop ?desig (for ?obj))
  ;;   (btr:object ?w ?ref-obj-name)
  ;;   (desig-location-prop ?ref-obj-name ?ref-obj-pose)
  ;;   (complete-object-size ?w ?ref-obj-name ?ref-obj-size)
  ;;   (padding-size ?w ?ref-obj-name ?ref-padding)
  ;;   (btr:object ?w ?for-obj-name)
  ;;   (complete-object-size ?w ?for-obj-name ?for-obj-size)
  ;;   (padding-size ?w ?for-obj-name ?for-padding)
  ;;   (-> (desig-prop ?desig (near ?ref-obj-name)
  ;;                   (pointed-costmap ?desig ?ref-obj-pose ?ref-obj-size ?ref-padding ?for-obj-size ?for-padding ?costmap))))
  
  ;; (<- (pointed-costmap ?desig ?ref-obj-pose ?ref-obj-size ?ref-padding ?for-obj-size ?for-padding ?costmap)
  ;;   (costmap  ?costmap)
  ;;   (instance-of gaussian-generator ?gaussian-generator-id)
  ;;   (costmap-add-function
  ;;    ?gaussian-generator-id
  ;;    (make-location-cost-function ?ref-obj-pose 1.0d0)
  ;;    ?costmap)
  ;; (lisp-fun calculate-near-costmap-min-radius ?ref-obj-size ?for-obj-size
  ;;             ?ref-padding ?for-padding ?min-radius)
  ;;   (costmap-width-in-obj-size-percentage-near ?cm-width-perc)
  ;;   (lisp-fun calculate-costmap-width ?ref-obj-size ?for-obj-size ?cm-width-perc
  ;;             ?cm-width)
  ;;   ;;
  ;;   (lisp-fun + ?min-radius ?cm-width ?max-radius)
  ;;   (instance-of range-generator ?range-generator-id-1)
  ;;   (costmap-add-function
  ;;    ?range-generator-id-1
  ;;    (make-range-cost-function ?ref-obj-pose ?max-radius)
  ;;    ?costmap)
  ;;   ;;
  ;;   (instance-of range-generator ?range-generator-id-2)
  ;;   (costmap-add-function
  ;;    ?range-generator-id-2
  ;;    (make-range-cost-function ?ref-obj-pose ?min-radius :invert t)
  ;;    ?costmap))

  ;;   ;; (findall ?obj (and
  ;;   ;;                (bullet-world ?world)
  ;;   ;;                (object ?world ?name)
  ;;   ;;                (%object ?world ?name ?obj)
  ;;   ;;                (lisp-type ?obj environment-object)
  ;;   ;;                (get-slot-value ?obj types ?types)
  ;;   ;;                (member ?type ?types)) ?objs)
  ;;   ;; (costmap-add-function collisions
  ;;   ;;                       (make-costmap-bbox-gen ?objs :invert t :padding 0.1)
  ;;   ;;                       ?costmap)
  ;;   (costmap ?costmap)
  ;;   (costmap-add-function reasoning-generator
  ;;                         (make-cognitive-reasoning-cost-function ?pos :Y  < 0.0)
  ;;                         ?costmap)
  ;;   (costmap ?costmap)
  ;;   (instance-of range-generator ?range-generator-id-1)
  ;;   (costmap-add-function ?range-generator-id-1
  ;;                         (make-range-cost-function ?pos 2.5)
  ;;                         ?costmap)
  ;;   (costmap ?costmap)
  ;;   (instance-of gaussian-generator ?gaussian-generator-id)
  ;;   (costmap-add-function ?gaussian-generator-id
  ;;                         (make-location-cost-function ?pos  1.0)
  ;;                         ?costmap))
   
 