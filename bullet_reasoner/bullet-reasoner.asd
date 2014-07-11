(defsystem bullet-reasoner
  :depends-on (roslisp 
	       cram-language
	       cognitive-reasoning)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "default" :depends-on ("package"))))))