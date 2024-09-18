export SIZE=250
python3 invertedai_carla_demo.py \
	--num-agents 30 \
	--location carla:Town10HD \
	--width $SIZE \
	--height $SIZE \
	--fov $SIZE \
	--map-center 0 30 \
	--sim-length 250  \
	--generationv All
