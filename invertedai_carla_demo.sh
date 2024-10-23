export SIZE=250
python3 invertedai_carla_demo.py \
	--number-of-vehicles 30 \
	--number-of-walkers 30 \
	--location carla:Town10HD \
	--width $SIZE \
	--height $SIZE \
	--fov $SIZE \
	--map-center 0 30 \
	--sim-length 120  \
	--safe \
	--record \
	--iai-key ... \
	--seed 5
