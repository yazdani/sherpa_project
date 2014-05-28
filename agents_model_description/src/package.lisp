(in-package :agents-knowledge)

(desig-props:def-desig-package agents-models-description
  (:nicknames :agents-knowledge)
  (:use #:common-lisp #:bullet-reasoning #:cram-reasoning)
  (:import-from  #:cram-manipulation-knowledge
                 )
  (:import-from #:cram-designators desig-prop)
  (:desig-properties type go-to look-at color))