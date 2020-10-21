#!/bin/sh

make && ./thnn_ceop \
  --problem=etc/tsiligirides_problem3/tsiligirides_problem_3_budget_050.txt \
  --communication-radius=1 \
  --heuristic=socp \
  --compute-matrix=0 \
  --iterations=10 \
  --repetitions=20