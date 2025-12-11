#!/bin/bash
FILES=("Startups.run_estimation" "Play.Walking.low_level" "Play.Walking.high_level" "Play.Walking.top_level")
ENV_NAME="bruce"
CONDA_SH="$HOME/anaconda3/etc/profile.d/conda.sh"

gnome-terminal --tab -- bash -i -c "gazebo --verbose Simulation/worlds/bruce.world"
sleep 5

/home/wyp/anaconda3/envs/bruce/bin/python3 -m "Startups.memory_manager"
sleep 1
/home/wyp/anaconda3/envs/bruce/bin/python3 -m "Simulation.sim_bruce" &
sleep 5

for FILE in "${FILES[@]}"; do
#   gnome-terminal --tab -- bash -i -c "source $CONDA_SH; conda run python3 -m $FILE -n bruce; exec bash"
  gnome-terminal --tab -- bash -i -c "./conda_act.sh $FILE"
  sleep 1
done

