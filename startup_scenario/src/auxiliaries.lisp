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


;;
;;Function for generating an action designator out of a command 
;;with the structure
;;

 (defun command-into-designator ()
   (cond ((equal nil *stored-result*) (format t "Ain't no publisher~%"))
         (t
          (let*((value (content *stored-result*))
                (command (read-from-string (LANGUAGE_INTERPRETER-MSG:COMMAND value)))
                (interpretation (read-from-string 
                                 (car 
                                  (split-sequence:split-sequence #\( (LANGUAGE_INTERPRETER-MSG:INTERPRETATION value)))))
                (selection (split-sequence:split-sequence #\Space (LANGUAGE_INTERPRETER-MSG:SELECTION value)))
                (agent-name (read-from-string (concatenate 'string (car selection) "_" (second selection)))))
	    (let*((vector (LANGUAGE_INTERPRETER-MSG:VEC value))
		  (value-x (GEOMETRY_MSGS-MSG:X vector))
		  (value-y (GEOMETRY_MSGS-MSG:Y vector))
		  (value-z (GEOMETRY_MSGS-MSG:Z vector)))
	      (let*((offset (LANGUAGE_INTERPRETER-MSG:OFFSET value))
		    (off-x (GEOMETRY_MSGS-MSG:X offset))
		    (off-y (GEOMETRY_MSGS-MSG:Y offset))
		    (off-z (GEOMETRY_MSGS-MSG:Z offset))))
	      (set-offset-checker off-x off-y off-z) 
	      (cond ((equal value-x 0.0d0)(equal value-y 0.0d0)(equal value-z 0.0d0)(setf vector (cl-transforms:make-pose (cl-transforms:make-3d-vector 0 0 0)
															       (cl-transforms:make-quaternion 0 0 0 1)))
		    (t 
		     (setf vector (cl-transforms:make-pose (cl-transforms:make-3d-vector 
							    (+ value-x off-x)  
							    (+ value-y off-y)
                                                            (+ value-z off-z))
                                                           (cl-transforms:make-quaternion 0 0 0 1)))
		     (setf *visualize-list* `((offset ,def)(loc ,gesture)))
                    (setf *act-desig* (make-designator 'action `((command_type ,command)
                                                                 (action_type ,interpretation)
                                                                 (offset ,(cl-transforms:make-3d-vector off-x off-y off-z))
                                                                 (agent ,agent-name)
                                                                 (target ,(make-designator 'location `((loc ,vector))))))))))))*act-desig*)))


(defun set-offset-checker (x y z)
  (cond ((not (equal x 0.0d0))(setf *offset-checker* 'x))
        ((not (equal y 0.0d0))(setf *offset-checker* 'y))
        (t
        (setf *offset-checker* 'z))))