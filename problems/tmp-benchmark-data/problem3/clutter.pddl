(define (domain clutter)
(:requirements :strips :equality)
(:predicates (on-surface ?x - objs ?surf - surface)
             (arm-empty)
             (holding ?x - objs)
             (on ?x - objs ?y - objs) )

(:action pickup
  :parameters (?ob - objs ?surf - surface)
  :precondition (and (on-surface ?ob ?surf) (arm-empty))
  :effect (and (holding ?ob) (not (on-surface ?ob ?surf)) 
               (not (arm-empty))))

(:action putdown
  :parameters (?ob - objs ?surf - surface)
  :precondition (holding ?ob)
  :effect (and (arm-empty) (on-surface ?ob ?surf) (not (holding ?ob))))

(:action stack
  :parameters (?ob - objs ?underob - objs)
  :precondition (holding ?ob)
  :effect (and (arm-empty) (on ?ob ?underob) (not (holding ?ob))))

(:action unstack
  :parameters (?ob - objs ?underob - objs)
  :precondition (and (on ?ob ?underob) (arm-empty))
  :effect (and (holding ?ob) (not (on ?ob ?underob)) (not (arm-empty)))))
                      
