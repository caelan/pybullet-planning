(define (problem pb4)
   (:domain blocksworld)
   (:objects 
     table red-tray blue-tray - surface
     a b c d e f g - objs )
   (:init 
          (on-surface a blue-tray) (on-surface b table) (on-surface c table)
	  (on d b) (on e c) (on f d) (on g e)
          (clear a) (clear g) (clear f)
          (arm-empty) )
   (:goal (or
    (and (on-surface a table) (on g f) (on f e) (on e d) (on d c) (on c b) (on b a))
    (and (on-surface a blue-tray) (on g f) (on f e) (on e d) (on d c) (on c b) (on b a))
    (and (on-surface a red-tray) (on g f) (on f e) (on e d) (on d c) (on c b) (on b a))
))

