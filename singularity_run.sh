#!/bin/bash
singularity exec -i --nv -n --network=none -p -B `pwd`:/jackal_ws/src/The-Barn-Challenge-Ros2 ${1} /bin/bash /jackal_ws/src/The-Barn-Challenge-Ros2/entrypoint.sh ${@:2}