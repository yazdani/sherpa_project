(defsystem agents-models-description
  :author "Fereshta Yazdani"
  :license "BSD"
  
  :depends-on (cram-reasoning
               designators
               cram-manipulation-knowledge
               bullet-reasoning)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "agents-knowledge" :depends-on ("package"))))))
