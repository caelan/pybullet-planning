(define (problem pb3)
   (:domain blocksworld)
   (:objects 
     table red-tray blue-tray - surface
     a b c - objs )
   (:init 
          (on-surface a blue-tray) (on-surface b table) (on-surface c table)  
          (clear a) (clear c) (clear d) 
          (arm-empty) )
   (:goal (or
    (and (on-surface a table) (on c b) (on b a))
    (and (on-surface a blue-tray) (on c b) (on b a))
    (and (on-surface a red-tray) (on c b) (on b a))
))

