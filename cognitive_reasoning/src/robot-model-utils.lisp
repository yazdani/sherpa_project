(in-package :cognitive-reasoning)

(defun calculate-pan-link (robot pan-link pose)
  "Calculates values for the pan link so that it poses on
  `pose'. Returns (LIST PAN-VALUE)"
  (let* ((pan-transform (cl-transforms:reference-transform
                         (link-pose robot pan-link)))
         (pose-trans (etypecase pose
                       (cl-transforms:3d-vector
                        (cl-transforms:make-transform
                         pose (cl-transforms:make-quaternion 0 0 0 1)))
                       (cl-transforms:pose (cl-transforms:reference-transform pose))
                       (cl-transforms:transform pose)))
         (pose-in-pan (cl-transforms:transform*
                       (cl-transforms:transform-inv pan-transform)
                       pose-trans))
         (pan-joint-name (cl-urdf:name
                          (cl-urdf:from-joint
                           (gethash pan-link (cl-urdf:links (urdf robot)))))))

    (format t "pan-transform: ~a~% pose-trans: ~a~% pose-in-pan: ~a~% pan-joint-name: ~a~%" pan-transform
            pose-trans pose-in-pan pan-joint-name)))