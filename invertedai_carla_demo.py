#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate realistic traffic with the InvertedAI API"""

import os
import time
import carla
import argparse
from invertedai_tools import *
import logging

SpawnActor = carla.command.SpawnActor

def argument_parser():

    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=20,
        type=int,
        help='Number of vehicles spawned by InvertedAI (default: 20)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=0,
        type=int,
        help='Number of walkers (default: 0)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='3',
        help='restrict to certain vehicle generation (values: "2","3","All" - default: "3")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='3',
        help='restrict to certain pedestrian generation (values: "2","3","All" - default: "3")')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--non-iai-ego',
        action='store_true',
        help="Spawn an ego vehicle not driven by InvertedAI simulation, but controlled by the standard traffic manager",
        default=False
    )
    argparser.add_argument(
        '--iai-key',
        type=str,
        help="InvertedAI API key."
    )
    argparser.add_argument(
        '--record',
        action='store_true',
        help="Record the simulation using the CARLA recorder",
        default=False
    )
    argparser.add_argument(
        '--sim-length',
        type=int,
        default=120,
        help="Length of the simulation in seconds (default: 120)",
    )
    argparser.add_argument(
        '--location',
        type=str,
        help=f"IAI formatted map on which to create simulate.",
        default='carla:Town10HD'
    )
    argparser.add_argument(
        '--capacity',
        type=int,
        help=f"The capacity parameter of a quadtree leaf before splitting.",
        default=100
    )
    argparser.add_argument(
        '--fov',
        type=int,
        help=f"Field of view for visualization.",
        default=100
    )
    argparser.add_argument(
        '--width',
        type=int,
        help=f"Full width of the area to initialize.",
        default=100
    )
    argparser.add_argument(
        '--height',
        type=int,
        help=f"Full height of the area to initialize",
        default=100
    )
    argparser.add_argument(
        '--map-center',
        type=int,
        nargs='+',
        help=f"Center of the area to initialize",
        default=tuple([0,0])
    )
    argparser.add_argument(
        '--is-async',
        type=bool,
        help=f"Whether to call drive asynchronously.",
        default=True
    )

    args = argparser.parse_args()

    return args

def spawn_pedestrians(client, world, num_pedestrians, bps):

    batch = []

    # Get spawn points for pedestrians
    spawn_points = []
    for i in range(num_pedestrians):
        
        loc = world.get_random_location_from_navigation()
        if (loc is not None):
            spawn_point = carla.Transform(location=loc)
            #Apply Offset in vertical to avoid collision spawning
            spawn_point.location.z += 1
            spawn_points.append(spawn_point)

    pedestrians = []
    walkers_list = []

    # Spawn pedestrians
    for i in range(len(spawn_points)):
        walker_bp = random.choice(bps)
        speed = walker_bp.get_attribute('speed').recommended_values[1]
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        spawn_point = spawn_points[i]
        batch.append(SpawnActor(walker_bp, spawn_point))

    results = client.apply_batch_sync(batch, True)
    pedestrians = world.get_actors().filter('walker.*')
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})

    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)

    world.tick()

    for controller in world.get_actors().filter('controller.ai.walker'):
        controller.start()
        dest = world.get_random_location_from_navigation()
        controller.go_to_location(dest)
        controller.set_max_speed(1 + random.random())

    return pedestrians

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3, 4]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


def set_spectator(world, hero_v):

    spectator_offset_x = -6.
    spectator_offset_z = 6.
    spectator_offset_pitch = 20

    hero_t = hero_v.get_transform()

    yaw = hero_t.rotation.yaw
    spectator_l = hero_t.location + carla.Location(
        spectator_offset_x * math.cos(math.radians(yaw)),
        spectator_offset_x * math.sin(math.radians(yaw)),
        spectator_offset_z,
    )
    spectator_t = carla.Transform(spectator_l, hero_t.rotation)
    spectator_t.rotation.pitch -= spectator_offset_pitch
    world.get_spectator().set_transform(spectator_t)


def main():

    args = argument_parser()

    # Specify your key here or through the IAI_API_KEY variable
    iai.add_apikey(args.iai_key)  

    num_pedestrians = args.number_of_walkers
    non_iai_ego = args.non_iai_ego

    FPS = 10
    
    client, world = setup_carla_environment(args)

    if args.record:
        logfolder = os.getcwd()+"/logs/"
        if not os.path.exists(logfolder):
            os.system("mkdir "+logfolder)
        logfile = logfolder+"record8.log"
        client.start_recorder(logfile)
        print("Recording on file: %s" % logfile)

    seed = args.seed

    if seed:
        random.seed(seed)
    
    vehicle_blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
    if args.safe:
        vehicle_blueprints = [x for x in vehicle_blueprints if x.get_attribute('base_type') == 'car']   

    agent_states, agent_properties = [], []
    is_iai = []
    noniai_actors = []

    # Add ego vehicle not driven by InvertedAI
    if non_iai_ego:
        print("Spawning an ego vehicle not driven by InvertedAI simulation, but controlled by the standard traffic manager")
        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_synchronous_mode(True)
        blueprint = random.choice(vehicle_blueprints)
        spawn_points = world.get_map().get_spawn_points()
        blueprint.set_attribute('role_name', 'hero')
        ego_vehicle = None
        while ego_vehicle is None:
            agent_transform = random.choice(spawn_points)
            ego_vehicle = world.try_spawn_actor(vehicle_blueprints[0],agent_transform)
        
        ego_state, ego_properties = initialize_iai_agent(ego_vehicle, "car")
        agent_states.append(ego_state)
        agent_properties.append(ego_properties)
        ego_vehicle.set_simulate_physics(False)
        ego_vehicle.set_autopilot(True)
        is_iai.append(False)
        noniai_actors.append(ego_vehicle)
        
        
    # Add pedestrians
    if num_pedestrians>0:
        if seed:
            world.set_pedestrians_seed(seed)
        blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)
        if not blueprintsWalkers:
            raise ValueError("Couldn't find any walkers with the specified filters")
        pedestrians = spawn_pedestrians(client, world, num_pedestrians, blueprintsWalkers)
        iai_pedestrians_states, iai_pedestrians_properties = initialize_pedestrians(pedestrians)
        agent_states.extend(iai_pedestrians_states)
        agent_properties.extend(iai_pedestrians_properties)
        is_iai.extend( [False]*len(iai_pedestrians_states) )
        noniai_actors.extend(pedestrians)
    else:
        pedestrians = []
    
    num_noniai = len(agent_properties)

    # Initialize InvertedAI co-simulation
    response, carla2iai_tl, location_info_response = initialize_simulation(args, world, agent_states=agent_states, agent_properties=agent_properties)
    agent_properties = response.agent_properties
    is_iai.extend( [True]*(len(agent_properties)-num_noniai) )

    log_writer = iai.LogWriter()
    log_writer.initialize(
        location=args.location,
        location_info_response=location_info_response,
        init_response=response
    )
    iailog_path = os.path.join(os.getcwd(),f"iailog.json")

    # Map IAI agents to CARLA actors and update response properties and states
    iai_to_carla_mapping, agent_properties, agent_states_new, recurrent_states_new = assign_carla_blueprints_to_iai_agents(world,vehicle_blueprints,agent_properties,response.agent_states,response.recurrent_states,is_iai,noniai_actors)
    traffic_lights_states = assign_iai_traffic_lights_from_carla(world,response.traffic_lights_states, carla2iai_tl)
    response.agent_states = agent_states_new
    response.recurrent_states = recurrent_states_new
    response.traffic_lights_states = traffic_lights_states

    carla_tick(iai_to_carla_mapping,response,world,is_iai)

    print("Is IAI",is_iai)

    try:

        vehicles = world.get_actors().filter('vehicle.*')
        print("Total number of agents:",len(agent_properties),"Vehicles",len(vehicles), "Pedestrians:",len(pedestrians))
        # print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))
        
        # Get hero vehicle
        hero_v = None
        if non_iai_ego:
            hero_v = ego_vehicle
        if hero_v is None:
            hero_v = vehicles[0]
        # hero_v = world.get_actors().filter('walker.*')[0]


        for frame in range(args.sim_length * FPS):

            response.traffic_lights_states = assign_iai_traffic_lights_from_carla(world, response.traffic_lights_states, carla2iai_tl)

            print(frame, len(response.agent_states),len(agent_properties),len(response.recurrent_states))

            # IAI update step
            response = iai.large_drive(
                location = args.location,
                agent_states = response.agent_states,
                agent_properties = agent_properties,
                recurrent_states = response.recurrent_states,
                traffic_lights_states = response.traffic_lights_states,
                light_recurrent_states = None,
                single_call_agent_limit = args.capacity,
                async_api_calls = args.is_async,
                api_model_version = "bI5p",
                random_seed = seed
            )

            log_writer.drive(drive_response=response)

            carla_tick(iai_to_carla_mapping,response,world,is_iai)

            # Update agents not driven by IAI, like pedestrians
            for agent_id in range(len(agent_properties)):
                if not is_iai[agent_id]:
                    actor = iai_to_carla_mapping[agent_id]
                    state, properties = initialize_iai_agent(actor, "car")
                    response.agent_states[agent_id] = state
                    agent_properties[agent_id] = properties

            # Update spectator view
            set_spectator(world, hero_v)

            

    finally:

        vehicles_list = world.get_actors().filter('vehicle.*')
        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        walkercontrollers_list = world.get_actors().filter('controller.*')
        for control in walkercontrollers_list:
            control.stop()
            control.destroy()

        walkers_list = world.get_actors().filter('walker.*')
        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in walkers_list])

        time.sleep(0.5)

        if args.record:
            client.stop_recorder()

        log_writer.export_to_file(log_path=iailog_path)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')