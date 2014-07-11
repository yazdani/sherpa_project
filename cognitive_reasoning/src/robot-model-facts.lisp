(in-package :cognitive-reasoning)

;; (def-fact-group robot-model (assert retract)
;; (<- (camera-pointing-at ?w ?robot-name ?pose)
;;       (format "CAMERA-POINTING-AT: ~a ~a~%" ?robot-name ?pose)
;;       (robot ?robot-name)
;;       (agents-model-description::robot-pan-links ?link)
;;       (bullet-world ?w)
;;       (%object ?w ?robot-name ?robot)
;;       (format "so das ist nun das ende~%")
;;       ;; (lisp-fun calculate-pan-link ?robot ?link ?pose
;;       ;; 		(?link-pos))
;;       ))