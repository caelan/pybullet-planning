(define (domain blocksworld)
(:requirements :strips :equality)
(:predicates (clear ?x - objs)
             (on-surface ?x - objs ?surf - surface)
             (arm-empty)
             (holding ?x - objs)
             (on ?x - objs ?y - objs) )

(:action pickup
  :parameters (?ob - objs ?surf - surface)
  :precondition (and (clear ?ob) (on-surface ?ob ?surf) (arm-empty))
  :effect (and (holding ?ob) (not (clear ?ob)) (not (on-surface ?ob ?surf)) 
               (not (arm-empty))))

(:action putdown
  :parameters (?ob - objs ?surf - surface)
  :precondition (and (holding ?ob))
  :effect (and (clear ?ob) (arm-empty) (on-surface ?ob ?surf) 
               (not (holding ?ob))))

(:action stack
  :parameters (?ob - objs ?underob - objs)
  :precondition (and  (clear ?underob) (holding ?ob))
  :effect (and (arm-empty) (clear ?ob) (on ?ob ?underob)
               (not (clear ?underob)) (not (holding ?ob))))

(:action unstack
  :parameters (?ob - objs ?underob - objs)
  :precondition (and (on ?ob ?underob) (clear ?ob) (arm-empty))
  :effect (and (holding ?ob) (clear ?underob)
               (not (on ?ob ?underob)) (not (clear ?ob)) (not (arm-empty)))))
                      
