(define (problem task_004)
  (:domain lego)

  (:objects
    brick_1_4x2 brick_2_4x2 - brick
  )

  (:init
    (on_table brick_1_4x2)
    (clear brick_1_4x2)
    (pick_area brick_1_4x2)
    (on_table brick_2_4x2)
    (clear brick_2_4x2)
    (pick_area brick_2_4x2)
    (hand_empty)
  )

  (:goal (and
    (assembly_area brick_1_4x2)
    (assembly_area brick_2_4x2)
    (on_table brick_1_4x2)
    (on_brick brick_2_4x2 brick_1_4x2)
  ))
)