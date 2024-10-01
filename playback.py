import carla
import os

client = carla.Client()
world = client.get_world()
world_settings = world.get_settings()
world_settings.fixed_delta_seconds = 0.1
world.apply_settings(world_settings)

logfile = os.getcwd()+"/logs/record.log"

id_vehicle = 31

print(client.show_recorder_file_info(logfile, True))
print(client.replay_file(logfile, 0, 0, id_vehicle, False))

hero_v = world.get_actor(id_vehicle)

camdir = "cameraout"
if not os.path.exists(camdir+"/"):
    os.system("mkdir "+camdir+"/")

# Spawn a camera
camera_init_trans = carla.Transform(carla.Location(x=1., z=1.5))
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '1920')
camera_bp.set_attribute('image_size_y', '1080')
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=hero_v)
camera.listen(lambda image: image.save_to_disk(camdir+'/%06d.png' % image.frame))



while True:
    world.wait_for_tick()
