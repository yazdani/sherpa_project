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

(desig-props:def-desig-package startup-scenario
  (:nicknames :startup-scenario); :cognitive-reasoning)
  (:use #:common-lisp   
        #:cognitive-reasoning
        #:roslisp
	;;        #:cpl
        #:cram-roslisp-common
        #:cram-designators
	;;	#:plan-lib
	;;        #:cram-plan-library
        ;; #:agents-model-description
        #:cram-agents-knowledge
        #:cognitive-reasoning
        #:location-costmap
        #:cram-reasoning
        #:cram-utilities
        #:btr
        #:cram-environment-representation
	#:agents-projection-process-modules)
  (:shadowing-import-from #:btr object household-object pose object-pose width height robot)
  (:shadowing-import-from #:cram-agents-knowledge quadrotor)
  ;(-from #:agents-model-description robot)
  (:import-from #:cram-reasoning #:<- #:def-fact-group)
  ;; (:import-from #:cram-agents-knowledge quadrotor);cram-pr2-knowledge pr2)
  (:export tree victim)
  (:desig-properties #:go-to #:far-from #:close-to #:sourrounded-by #:right-of #:left-of #:type #:behind #:in-front #:for-robot #:name #:color #:a-gesture #:pointed-pos))
