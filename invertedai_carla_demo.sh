export SIZE=250
python3 invertedai_carla_demo.py \
	--number-of-vehicles 50 \
	--number-of-walkers 50 \
	--location carla:Town10HD \
	--width $SIZE \
	--height $SIZE \
	--fov $SIZE \
	--map-center 0 30 \
	--sim-length 60  \
	--safe \
	--record \
	--iai-key PhrisSyKXpHmrLRguFYA79vnAtfCeNh3TnTY1Mg3 \
	--seed 5
