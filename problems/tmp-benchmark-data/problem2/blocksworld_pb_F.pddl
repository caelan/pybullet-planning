(define (problem pb4)
   (:domain blocksworld)
   (:objects 
     table red-tray blue-tray - surface
     a b c d e f - objs )
   (:init 
          (on-surface a blue-tray) (on-surface b table) (on-surface c table)
	  (on d b) (on e c) (on f d)
          (clear a) (clear e) (clear f)
          (arm-empty) )
   (:goal (or
    (and (on-surface a table) (on f e) (on e d) (on d c) (on c b) (on b a))
    (and (on-surface a blue-tray) (on f e) (on e d) (on d c) (on c b) (on b a))
    (and (on-surface a red-tray) (on f e) (on e d) (on d c) (on c b) (on b a))
))

