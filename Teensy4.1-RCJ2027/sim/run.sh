#!/bin/bash
# Build one act program for the PC and run the scenarios.
# usage: ./run.sh [program] [scenario | all]
#   ./run.sh                              -> act_defense_smach, all scenarios
#   ./run.sh act_defense_smach ball_front -> one scenario
# Output: sim/out/<program>_<scenario>.html (open in a browser)
set -e
cd "$(dirname "$0")"
PROG=${1:-act_defense_smach}
SCEN=${2:-all}
MUX2=0
[[ $PROG == *defense* ]] && MUX2=1            # defense robot: mux 2 line sensors

mkdir -p out
g++ -std=gnu++17 -O2 -w -fpermissive -Istub -I../lib/core_act -I../include \
    -DSIM_MUX2=$MUX2 -DSIM_PROG="\"$PROG\"" \
    ../lib/core_act/core_act.cpp ../src/$PROG.cpp fake_hw.cpp world.cpp -o out/sim_$PROG
./out/sim_$PROG $SCEN
