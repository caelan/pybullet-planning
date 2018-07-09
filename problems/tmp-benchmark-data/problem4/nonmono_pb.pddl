(define (problem pb7)
   (:domain clutter)
   (:objects 
     table1 table2 rt1 rt2 rt3 rt4 bt1 bt2 bt3 bt4 gt1 gt2 gt3 - surface
     rs1 rs2 rs3 rs4 bs1 bs2 bs3 bs4 gs1 gs2 gs3 - objs )
   (:init 
          (on-surface rs1 rt1) (on-surface rs2 rt2) (on-surface rs3 rt3) (on-surface rs4 rt4) 
          (on-surface gs1 table1) (on-surface gs2 table1) (on-surface gs3 table1)
          (on-surface bs1 bt1) (on-surface bs2 bt2) (on-surface bs3 bt3) (on-surface bs4 bt4) 
          (arm-empty) )
   (:goal (and
          (on-surface rs1 rt1) (on-surface rs2 rt2) (on-surface rs3 rt3) (on-surface rs4 rt4) 
          (on-surface gs1 gt1) (on-surface gs2 gt2) (on-surface gs3 gt3)
          (on-surface bs1 bt1) (on-surface bs2 bt2) (on-surface bs3 bt3) (on-surface bs4 bt4) 
))

