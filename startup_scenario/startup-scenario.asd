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

(defsystem startup-scenario
  :author "Fereshta Yazdani <yazdani@cs.uni-bremen.de>"
  :license "BSD"
  :description "STARTUP-SCENARIO"
   :depends-on (cram-language
               location-costmap
               roslisp
	       simple-knowledge
	       ;; agents-model-description
	       ;; pr2-manipulation-knowledge
	       simple-knowledge
	       cram-environment-representation
	       ;; projection-process-modules
	       cram-gazebo-utilities
	       bullet-reasoning-designators
               control_msgs-msg
               geometry_msgs-msg
               trajectory_msgs-msg
	       cognitive-reasoning
	       cram-environment-representation
	       actionlib
            
	       gazebo-perception-process-module
	       gazebo_msgs-msg
	       agents-projection-process-modules
	       semantic-map-costmap
	       designator-integration-lisp
	       household_objects_database_msgs-msg)
 :components 
  ((:module "src"
    :components
    ((:file "package")
     (:file "start-scenario" :depends-on ("package" "calculation" "auxiliaries" "ros-interactor"))
     (:file "auxiliaries" :depends-on ("package"))
     (:file "calculation" :depends-on ("package"))
     (:file "ros-interactor" :depends-on ("package"))
     ))))
