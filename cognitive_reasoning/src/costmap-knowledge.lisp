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
(def-fact-group costmap-metadata ()
  (<- (costmap-size 30 30))
  (<- (costmap-origin -15 -15))
  (<- (costmap-resolution 0.025))

  ;; (<- (costmap-size 10 10))
  ;; (<- (costmap-origin -145 -160))
  ;; (<- (costmap-resolution 2))

  (<- (costmap-padding 0.38))
  (<- (costmap-manipulation-padding 0.38))
  (<- (costmap-in-reach-distance 0.9))
  (<- (costmap-reach-minimal-distance 0.2)))

(def-fact-group semantic-map-data (semantic-map-name)
  (<- (cl-semantic-map-utils::semantic-map-name
       "http://knowrob.org/kb/ias_semantic_map.owl#SemanticEnvironmentMap_PM582j"))
  (<- (semantic-map-obj sem-map)))

(disable-location-validation-function 'btr-desig::robot-location-on-floor)
(disable-location-validation-function 'designators-ros::filter-solution)
;; (disable-location-validation-function 'btr-desig::validate-designator-solution)
(disable-location-validation-function 'btr-desig::check-ik-solution)
;; (disable-location-validation-function 'desig::designator-prolog-desig-solution-validator)
;; (disable-location-validation-function 'location-costmap::location-costmap-pose-validator)
