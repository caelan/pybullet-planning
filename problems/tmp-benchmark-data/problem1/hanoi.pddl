;; The Towers of Hanoi problem (formalisation by Hector Geffner).

(define (domain hanoi)
  (:requirements :strips)
  (:predicates (clear ?x) (on ?x ?y) (smaller ?x ?y))

  (:action move
    :parameters (?disc ?from ?to)
    :precondition (and (smaller ?to ?disc) (on ?disc ?from) 
		       (clear ?disc) (clear ?to))
    :effect  (and (clear ?from) (on ?disc ?to) (not (on ?disc ?from))  
		  (not (clear ?to))))
)
