# InvertedAI_CARLA_demo

Demo to showcase the integration of the [InvertedAI](https://www.inverted.ai/home) API for data-driven realistic traffic generation in the autonomous driving [CARLA simulator](https://carla.org/).

## Usage

- Run with `python3 invertedai_carla_demo.py --iai-key API_KEY`, where `API_KEY` is the required InvertedAI API key, or `sh invertedai_carla_demo.sh`.

- Record the simulation with the CARLA recorder with the flag `--record`, to replay it afterwards with `python3 playback.py`.

- Generate an InvertedAI log file enabling the flag `--iai-log` and for reading, analyzing and generating a gif with `python3 iai_log_reader.py`.

<img src="iai_junction.gif" width="100%" alt="InvertedAI simulation running in CARLA.">
