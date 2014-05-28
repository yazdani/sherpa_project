;;; Copyright (c) 2014, Fereshta Yazdani <yazdani@cs.uni-bremen.de>
;;; All rights reserved.
;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to
;;; endorse or promote products derived from this software without
;;; specific prior written permission.
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

(in-package :agents-knowledge)
;;;TODO: FIXING THE CAM-PROPERTIES OF DIFFERENT AGENTS
;;;TODO SRDL FOR HUMAN
;; (def-fact-group human-metadata (robot cam-frame)
;;   (<- (robot human))
;;   (<- (cam-frame human "camera_rgb_frame")))

(def-fact-group quadrotor-metadata (robot camera-frame camera-minimal-height camera-maximal-height)
  (<- (robot quadrotor))
  (<- (camera-frame quadrotor "camera_rgb_optical_frame"))
  (<- (camera-frame quadrotor "camera_depth_optical_frame"))
  (<- (camera-minimal-height 0.6))
  (<- (camera-maximal-height 1.0)))

;; (def-fact-group ground-robot-metadata (robot camera-frame camera-minimal-height camera-maximal-height)
;;   (<- (robot quadrotor))
;;   (<- (camera-frame quadrotor "camera_rgb_optical_frame"))
;;   (<- (camera-frame quadrotor "camera_depth_optical_frame")))