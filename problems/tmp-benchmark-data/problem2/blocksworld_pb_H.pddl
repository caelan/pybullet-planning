(define (problem pb4)
   (:domain blocksworld)
   (:objects 
     table red-tray blue-tray - surface
     a b c d e f g h - objs )
   (:init 
          (on-surface a blue-tray) (on-surface b table) (on-surface c table)
	  (on d b) (on e c) (on f d) (on g e) (on h f)
          (clear a) (clear g) (clear h)
          (arm-empty) )
   (:goal (or
    (and (on-surface a table) (on h g) (on g f) (on f e) (on e d) (on d c) (on c b) (on b a))
    (and (on-surface a blue-tray) (on h g) (on g f) (on f e) (on e d) (on d c) (on c b) (on b a))
    (and (on-surface a red-tray) (on h g) (on g f) (on f e) (on e d) (on d c) (on c b) (on b a))
))

