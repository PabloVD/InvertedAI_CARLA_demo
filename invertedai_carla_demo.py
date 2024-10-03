#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate realistic traffic with the InvertedAI API"""

import os
import time
import math
import carla
import argparse
from numpy import random
from invertedai_tools import *


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
        default=100
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

def spawn_pedestrians(world, num_pedestrians, filterw, generationw):

    bps = get_actor_blueprints(world, filterw, generationw)
    spawn_points = world.get_map().get_spawn_points()

    pedestrians = []

    for i in range(num_pedestrians):
        bp = random.choice(bps)
        spawn_point = random.choice(spawn_points)
        actor = world.try_spawn_actor(bp,spawn_point)
        if actor is not None:
            pedestrians.append(actor)
    
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

    num_pedestrians = args.w
    non_iai_ego = args.non_iai_ego

    FPS = 10
    
    client, world = setup_carla_environment(args)

    tl_lights = [tl.id for tl in list(world.get_actors().filter('traffic.traffic_light*'))]
    print(tl_lights)

    if args.record:
        logfolder = os.getcwd()+"/logs/"
        if not os.path.exists(logfolder):
            os.system("mkdir "+logfolder)
        logfile = logfolder+"record4.log"
        client.start_recorder(logfile)
        print("Recording on file: %s" % logfile)

    random.seed(args.seed if args.seed is not None else int(time.time()))

    vehicles_list = []
    walkers_list = []
    all_id = []
    
    iai_seed = args.seed #random.randint(1,10000)

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
        pedestrians = spawn_pedestrians(world, num_pedestrians, args.filterw, args.generationw)
        iai_pedestrians_states, iai_pedestrians_properties = initialize_pedestrians(pedestrians)
        agent_states.extend(iai_pedestrians_states)
        agent_properties.extend(iai_pedestrians_properties)
        is_iai.extend( [False]*len(iai_pedestrians_states) )
        noniai_actors.extend(pedestrians)
    
    num_noniai = len(agent_properties)


    # Initialize InvertedAI co-simulation
    response = initialize_simulation(args, world, agent_states=agent_states, agent_properties=agent_properties)
    agent_properties = response.agent_properties
    is_iai.extend( [True]*(len(agent_properties)-num_noniai) )

    # Map IAI agents to CARLA actors and update response properties and states
    iai_to_carla_mapping, agent_properties, agent_states_new, recurrent_states_new = assign_carla_blueprints_to_iai_agents(world,vehicle_blueprints,agent_properties,response.agent_states,response.recurrent_states,is_iai,noniai_actors)
    traffic_lights_states = assign_iai_traffic_lights_from_carla(world,response.traffic_lights_states)
    response.agent_states = agent_states_new
    response.recurrent_states = recurrent_states_new
    response.traffic_lights_states = traffic_lights_states

    carla_tick(iai_to_carla_mapping,response,world,is_iai)

    try:

        # if not blueprints:
        #     raise ValueError("Couldn't find any vehicles with the specified filters")
        blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)
        if not blueprintsWalkers:
            raise ValueError("Couldn't find any walkers with the specified filters")


        vehicles = world.get_actors().filter('vehicle.*')
        print("IAI vehicles:",len(agent_properties),"CARLA vehicles",len(vehicles))

        vehicles = world.get_actors().filter('vehicle.*')

        batch = []
        
        # Get hero vehicle
        if non_iai_ego:
            hero_v = ego_vehicle
        if hero_v is None:
            hero_v = possible_vehicles[0]

        # spawn_points = []
        # hero_start_point = ego_wp.transform.location
        # hero_end_point = ego_wp.next(pedestrian_point)[0].transform.location
        # for i in range(num_pedestrians):
        #     spawn_point = carla.Transform()
        #     loc = None
        #     while loc is None:
        #         loc = world.get_random_location_from_navigation()
        #         if loc.distance(hero_start_point) < pedestrian_dist or loc.distance(hero_end_point) < pedestrian_dist:
        #             spawn_point.location = loc
        #             spawn_point.location.z += 2
        #             spawn_points.append(spawn_point)
        #         else:
        #             loc = None

        # batch = []
        # walker_speed = []
        # for spawn_point in spawn_points:
        #     walker_bp = random.choice(blueprintsWalkers)
        #     if walker_bp.has_attribute('is_invincible'):
        #         walker_bp.set_attribute('is_invincible', 'false')
        #     if walker_bp.has_attribute('speed'):
        #         walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
        #     else:
        #         print("Walker has no speed")
        #         walker_speed.append(0.0)
        #     batch.append(carla.command.SpawnActor(walker_bp, spawn_point))
        # results = client.apply_batch_sync(batch, True)
        # walker_speed2 = []
        # for i in range(len(results)):
        #     if results[i].error:
        #         logging.error(results[i].error)
        #     else:
        #         walkers_list.append({"id": results[i].actor_id})
        #         walker_speed2.append(walker_speed[i])
        # walker_speed = walker_speed2

        # batch = []
        # walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        # for i in range(len(walkers_list)):
        #     batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        # results = client.apply_batch_sync(batch, True)
        # for i in range(len(results)):
        #     if results[i].error:
        #         logging.error(results[i].error)
        #     else:
        #         walkers_list[i]["con"] = results[i].actor_id

        # for i in range(len(walkers_list)):
        #     all_id.append(walkers_list[i]["con"])
        #     all_id.append(walkers_list[i]["id"])
        # all_actors = world.get_actors(all_id)

        

        # world.tick()

        # world.set_pedestrians_cross_factor(0.0)
        # for i in range(0, len(all_id), 2):
        #     all_actors[i].start()
        #     all_actors[i].go_to_location(world.get_random_location_from_navigation())
        #     all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        # print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        
        for frame in range(args.sim_length * FPS):

            response.traffic_lights_states = assign_iai_traffic_lights_from_carla(world,response.traffic_lights_states)

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
                random_seed = iai_seed
            )

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

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        walkers_list = world.get_actors().filter('walker.*')
        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        time.sleep(0.5)

        if args.record:
            client.stop_recorder()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')