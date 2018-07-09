(define (problem pb7)
   (:domain clutter)
   (:objects 
     table1 table2 table3 table4 - surface
     bs1 bs2 bs3 bs4 bs5 bs6 bs7 gs1 gs2 gs3 gs4 gs5 gs6 gs7 rs1 rs2 rs3 rs4 rs5 rs6 rs7 rs8 rs9 rs10 rs11 rs12 rs13 rs14 - objs )
   (:init 
          (on-surface bs1 table1) (on-surface bs2 table1) (on-surface bs3 table1) 
          (on-surface gs1 table1) (on-surface gs2 table1) (on-surface gs3 table1) (on-surface gs4 table1) 
          (on-surface rs1 table1) (on-surface rs2 table1) (on-surface rs3 table1) (on-surface rs4 table1) 
          (on-surface rs5 table1) (on-surface rs6 table1) (on-surface rs7 table1)
          (on-surface bs4 table2) (on-surface bs5 table2) (on-surface bs6 table2) (on-surface bs7 table2) 
          (on-surface gs5 table2) (on-surface gs6 table2) (on-surface gs7 table2)
          (on-surface rs8 table2) (on-surface rs9 table2) (on-surface rs10 table2) (on-surface rs11 table2) 
          (on-surface rs12 table2) (on-surface rs13 table2) (on-surface rs14 table2)
          (arm-empty) )
   (:goal (and
    (on-surface bs1 table3) (on-surface bs2 table3) (on-surface bs3 table3) (on-surface bs4 table3)
    (on-surface bs5 table3) (on-surface bs6 table3) (on-surface bs7 table3) 
    (on-surface gs1 table4) (on-surface gs2 table4) (on-surface gs3 table4) (on-surface gs4 table4) 
    (on-surface gs5 table4) (on-surface gs6 table4) (on-surface gs7 table4)
))

