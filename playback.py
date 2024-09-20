import carla
import os

client = carla.Client()
world = client.get_world()
world_settings = world.get_settings()
world_settings.fixed_delta_seconds = 0.1
world.apply_settings(world_settings)

logfile = os.getcwd()+"/logs/record.log"

print(client.show_recorder_file_info(logfile, True))

id_vehicle = 47

print(client.replay_file(logfile, 0, 0, id_vehicle, False))