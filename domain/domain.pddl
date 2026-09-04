(define (domain lego)
  (:requirements :strips :typing)
  
  (:types brick)

  (:predicates
    (on_brick ?b1 - brick ?b2 - brick)
    (on_table ?b - brick)
    (clear ?b - brick)
    (holding ?b - brick)
    (pick_area ?b - brick)
    (assembly_area ?b - brick)
    (hand_empty)
  )

  (:action pick 
    :parameters (?b - brick)
    :precondition (and 
                    (hand_empty)
                    (pick_area ?b))
    :effect (and
              (not (pick_area ?b))
              (not (hand_empty))
              (holding ?b)
              (not (on_table ?b))))
    
  (:action place
    :parameters (?b - brick)
    :precondition (and 
                    (holding ?b))
    :effect (and
              (not (holding ?b))
              (assembly_area ?b)
              (hand_empty)
              (on_table ?b)))

  (:action stack
    :parameters (?b1 - brick ?b2 - brick)
    :precondition (and 
                    (holding ?b1) 
                    (clear ?b2) 
                    (assembly_area ?b2))
    :effect (and
              (not (holding ?b1))
              (assembly_area ?b1)
              (not (clear ?b2))
              (hand_empty)
              (on_brick ?b1 ?b2)))
)