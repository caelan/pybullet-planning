(define (problem pb4)
   (:domain blocksworld)
   (:objects 
     table red-tray blue-tray - surface
     a b c d e f g h i j - objs )
   (:init 
          (on-surface a blue-tray) (on-surface b table) (on-surface c table)
	  (on d b) (on e c) (on f d) (on g e) (on h f) (on i g) (on j h)
          (clear a) (clear i) (clear j)
          (arm-empty) )
   (:goal (or
    (and (on-surface a table) (on j i) (on i h) (on h g) (on g f) (on f e) (on e d) (on d c) (on c b) (on b a))
    (and (on-surface a blue-tray) (on j i) (on i h) (on h g) (on g f) (on f e) (on e d) (on d c) (on c b) (on b a))
    (and (on-surface a red-tray) (on j i) (on i h) (on h g) (on g f) (on f e) (on e d) (on d c) (on c b) (on b a))
))

