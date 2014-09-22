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

(defsystem cognitive-reasoning
  :author "Fereshta Yazdani <yazdani@cs.uni-bremen.de>"
  :license "BSD"
  :description "SHERPA SPATIAL RELATIONS"
  
  :depends-on (cram-language
               location-costmap
               roslisp
               ;; cram-pr2-knowledge
               ;; pr2-manipulation-knowledge
               ;; simple-knowledge
               ;; cram-gazebo-utilities
               ;;               cram-plan-library
               ;;               pr2-manipulation-process-module
               ;; cram-environment-representation
               ;;               object-location-designators
               ;;               cram-plan-knowledge
               ;; projection-process-modules
               ;; cram-gazebo-utilities
               ;;              simple-knowledge
	       cram-agents-knowledge
               bullet-reasoning-designators
               control_msgs-msg
               geometry_msgs-msg
               trajectory_msgs-msg
	       gazebo_msgs-msg
               household_objects_database_msgs-msg
               ;; household_objects_database_msgs-srv
               actionlib)
  :components 
  ((:module "src"
    :components
    ((:file "package")
     (:file "environment-objects" :depends-on ("package"))
     (:file "perception" :depends-on ("package"))
     (:file "cost-functions" :depends-on ("package"))
     (:file "prolog" :depends-on ("package" "cost-functions"))
     ;; (:file "robot-model-facts" :depends-on ("package" "robot-model-utils"))
     ;; (:file "robot-model-utils" :depends-on ("package"))
     (:file "costmap-knowledge" :depends-on("package"))
     ;; (:file "test-world" :depends-on("package" "prolog" "costmap-knowledge"))
     ;; (:file "environment-objects" :depends-on("package")) 
     ;; (:file "pointing-gesture" :depends-on("package"))
     ;; 	   (:file "calculations" :depends-on("package"))
     ))))
;;	    (:file "human-specific-objects" :depends-on("package"))
;;      (:file "designator-integration" :depends-on("package"))
;,      ))))