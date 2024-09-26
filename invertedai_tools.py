import invertedai as iai
from invertedai.common import AgentProperties, AgentState, StaticMapActor, TrafficLightState
import carla

import argparse
from tqdm import tqdm
import matplotlib.pyplot as plt
import time
import random
import math
import json

iai.add_apikey('')  # specify your key here or through the IAI_API_KEY variable

z_offset = 0.05

with open('carla2iai_ue5.json') as file:
    carla2iai = json.load(file)

def initialize_tl_states(world):
    iai_tl_states = {}
    for tlpair in carla2iai.values():
        for tl in tlpair:
            iai_tl_states[tl] = TrafficLightState.red # Initialize to given value

    iai_tl_states = assign_iai_traffic_lights_from_carla(world, iai_tl_states)
    return iai_tl_states

# Initialize IAI agents from CARLA actors
def initialize_iai_agent(actor, agent_type):

    transf = actor.get_transform()
    vel = actor.get_velocity()
    speed = math.sqrt(vel.x**2. + vel.y**2. +vel.z**2.)

    agent_state = AgentState.fromlist([
                                        transf.location.x,
                                        transf.location.y,
                                        transf.rotation.yaw,
                                        speed
                                    ])

    bb = actor.bounding_box
    length, width = bb.extent.x*2, bb.extent.y*2

    agent_properties = AgentProperties(length=length, width=width, agent_type=agent_type)
    if agent_type=="car":
        agent_properties.rear_axis_offset = length*0.38 # Empirical value fitted from InvertedAI initialization

    return agent_state, agent_properties

# Initialize IAI pedestrians from CARLA actors
def initialize_pedestrians(pedestrians):

    iai_pedestrians_states, iai_pedestrians_properties = [], []
    for actor in pedestrians:
        iai_ped_state, iai_ped_properties = initialize_iai_agent(actor,agent_type="pedestrian")
        iai_pedestrians_states.append(iai_ped_state)
        iai_pedestrians_properties.append(iai_ped_properties)

    return iai_pedestrians_states, iai_pedestrians_properties

def setup_carla_environment(args):
    step_length = 0.1 #0.1 is the only step length that is supported at this time

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)

    # Configure the simulation environment
    #world = client.load_world(args.location.split(":")[-1])
    world = client.get_world()
    world_settings = carla.WorldSettings(
        synchronous_mode=True,
        fixed_delta_seconds=step_length,
    )
    world.apply_settings(world_settings)

    return client, world

def get_vehicle_blueprint_list(world):
    # A method that returns a list of Carla vehicle blueprints based on any desireable criteria such as vehicle class, size, etc.

    # print(bp for bp in world.get_blueprint_library())

    # REPLACE THIS LIST WITH A LIST OF DESIRED VEHICLE BLUEPRINTS
    # blueprint_list = [
    #     world.get_blueprint_library().filter(bp_str) for bp_str in [
    #         "vehicle.audi.a2",
    #         "vehicle.audi.etron",
    #         "vehicle.audi.tt",
    #         "vehicle.bmw.grandtourer",
    #         "vehicle.citroen.c3",
    #         "vehicle.chevrolet.impala",
    #         "vehicle.dodge.charger_2020",
    #         "vehicle.ford.mustang",
    #         "vehicle.ford.crown",
    #         "vehicle.jeep.wrangler_rubicon",
    #         "vehicle.lincoln.mkz_2020",
    #         "vehicle.mercedes.coupe_2020",
    #         "vehicle.nissan.micra",
    #         "vehicle.nissan.patrol_2021",
    #         "vehicle.seat.leon",
    #         "vehicle.toyota.prius",
    #         # "vehicle.volkswagen.t2_2021",
    #         # "vehicle.tesla.cybertruck",
    #         "vehicle.tesla.model3",
    #         "vehicle.mini.cooper_s"
    #     ]
    # ]

    # world.get_blueprint_library().find(bp_str)

    blueprint_list = world.get_blueprint_library().filter("vehicle*")
    # print(blueprint_list)

    return blueprint_list

# Get a dictionary containing the length and width of the bounding box for each vehicle blueprint
def get_blueprint_dictionary(world, client):

    spawn_points = world.get_map().get_spawn_points()

    # Replace this list with the required blueprint filters, e.g., *audi* for audi vehicles, or "vehicle.audi.etron" for an specific model
    blueprint_filters = ["vehicle*"]
    # blueprint_filters = get_vehicle_blueprint_list(world)

    blueprint_list = []
    for bp_filter in blueprint_filters:
        blueprint_list.extend(world.get_blueprint_library().filter(bp_filter))

    # blueprint_list = [x for x in blueprint_list if x.get_attribute('base_type') == 'car']

    bp_dict = {}

    for bp in blueprint_list:

        spawn_point = random.choice(spawn_points)
        spawn_points.remove(spawn_point)
        actor = world.try_spawn_actor(bp, spawn_point)
        
        if actor is not None:
            bb = actor.bounding_box

            length, width = bb.extent.x*2, bb.extent.y*2

            bp_dict[bp] = [length, width]

            actor.destroy()

    # Destroy all vehicles spawned, since some of them may not be actually removed
    vehicles = world.get_actors().filter('vehicle.*')
    client.apply_batch([carla.command.DestroyActor(x) for x in vehicles])

    world.tick()

    return bp_dict

# Get the blueprint whose bounding box dimensions matches the provided length and width as close as possible
def get_closest_blueprint(bp_dict, length, width):

    bp = min(bp_dict, key=lambda bp: math.sqrt((bp_dict[bp][0]-length)**2. + (bp_dict[bp][1]-width)**2.))

    return bp

def transform_iai_to_carla(agent_state):
    agent_transform = carla.Transform(
        carla.Location(
            agent_state.center.x,
            agent_state.center.y,
            z_offset
        ),
        carla.Rotation(
            yaw=math.degrees(agent_state.orientation)
        )
    )

    return agent_transform

def match_carla_actor(attributes,bp_dict):
    # A method for making the specific selection of Carla vehicle blueprint to match an IAI agent
    # For example, the method could be to select the blueprint that most closely resembles the IAI vehicle attributes

    # REPLACE THIS BLUEPRINT SELECTION WITH A METHOD OF YOUR CHOICE
    #blueprint_selection = random.choice(blueprints)

    blueprint_selection = get_closest_blueprint(bp_dict, attributes.length, attributes.width)

    return blueprint_selection

def assign_carla_blueprints_to_iai_agents(world,vehicle_blueprints,agent_properties,agent_states,recurrent_states,is_iai,noniai_actors):
    # A method to assign existing IAI agents to a Carla vehicle blueprint and add this agent to the Carla simulation
    # Return a dictionary of IAI agent ID's mapped to references to Carla agents within the Carla environment (e.g. actor ID's)

    iai_to_carla_mapping = {}
    agent_properties_new = []
    agent_states_new = []
    recurrent_states_new = []
    new_agent_id = 0

    for agent_id, (state, attr) in enumerate(zip(agent_states,agent_properties)):

        if not is_iai[agent_id]:
            agent_properties_new.append(agent_properties[agent_id])
            agent_states_new.append(agent_states[agent_id])
            recurrent_states_new.append(recurrent_states[agent_id])
            actor = noniai_actors[agent_id]
            iai_to_carla_mapping[new_agent_id] = actor
            new_agent_id += 1
            
            
        else:

            #blueprint = match_carla_actor(attr,vehicle_blueprints)
            blueprint = random.choice(vehicle_blueprints)
            agent_transform = transform_iai_to_carla(state)

            print("BP and actor",agent_id,blueprint,agent_transform)

            actor = world.try_spawn_actor(blueprint,agent_transform)

            print(actor)
            
            if actor is not None:
                bb = actor.bounding_box.extent

                agent_attr = agent_properties[agent_id]

                agent_attr.length = 2*bb.x
                agent_attr.width = 2*bb.y
                agent_attr.rear_axis_offset = 2*bb.x/3

                iai_to_carla_mapping[new_agent_id] = actor
                new_agent_id += 1

                agent_properties_new.append(agent_attr)
                agent_states_new.append(agent_states[agent_id])
                recurrent_states_new.append(recurrent_states[agent_id])

                # if not is_iai[agent_id]:
                # print(agent_id, recurrent_states[agent_id])

                actor.set_simulate_physics(False)

        # print(agent_id, actor)

    if len(agent_properties_new) == 0:
        raise Exception("No vehicles could be placed in Carla environment.")
    
    print("ids",agent_id, new_agent_id)

    return iai_to_carla_mapping, agent_properties_new, agent_states_new, recurrent_states_new

def get_traffic_light_state_from_carla(carla_tl_state):
    # Returns carla traffic light state based on iai traffic light state.

    if carla_tl_state == carla.TrafficLightState.Red:
        return TrafficLightState.red

    elif carla_tl_state == carla.TrafficLightState.Yellow:
        return TrafficLightState.yellow

    elif carla_tl_state == carla.TrafficLightState.Green:
        return TrafficLightState.green

    else:  # Unknown state, turn off traffic light
        return TrafficLightState.Off

def get_traffic_light_state_from_iai(iai_tl_state,carla_tl_state):
    # Returns carla traffic light state based on iai traffic light state.

    if iai_tl_state == TrafficLightState.red:
        return carla.TrafficLightState.Red

    elif iai_tl_state == TrafficLightState.yellow:
        return carla.TrafficLightState.Yellow

    elif iai_tl_state == TrafficLightState.green:
        return carla.TrafficLightState.Green

    else:  # Unknown state, turn off traffic light
        return carla.TrafficLightState.Off

def get_carla_tl_id(iai_tl_id):
    # Define a map between IAI traffic light ID's and Carla traffic light ID's
    pass

def get_carla_traffic_lights(world):
    # A method for acquiring references to carla traffic light objects from the world

    carla_lights = {}
    for light in list(world.get_actors().filter('traffic.traffic_light*')):
        carla_lights[light.id] = light

    return carla_lights

# def set_traffic_lights_from_iai(traffic_lights_states,carla_lights):
#     # Assume carla_lights is a dictionary containing ID's of traffic light actors mapped to Carla traffic light agents
#     # This function is only a suggestion about how to set traffic lights
#     for tl_id, state in traffic_lights_states.items():
#         carla_tl_id = get_carla_tl_id(tl_id)
#         carla_lights[carla_tl_id].set_light(get_traffic_light_state_from_iai(state))

def set_traffic_lights_from_carla(iai_tl,carla_tl):
    # Assume carla_lights is a dictionary containing ID's of traffic light actors mapped to Carla traffic light agents
    for carla_tl_id, carla_state in carla_tl.items():
        iai_tl_id_pair = carla2iai[carla_tl_id]
        for iai_tl_id in iai_tl_id_pair:
            iai_tl[iai_tl_id] = get_traffic_light_state_from_carla(carla_state)
    return iai_tl

def assign_iai_traffic_lights_from_carla(world,iai_tl):

    traffic_lights = world.get_actors().filter('traffic.traffic_light*')
    
    carla_tl_dict = {}
    for tl in traffic_lights:
        carla_tl_dict[str(tl.id)]=tl.state

    return set_traffic_lights_from_carla(iai_tl,carla_tl_dict)

def carla_tick(iai_to_carla_mapping,response,world,is_iai):
    """
    Tick the carla simulation forward one time step
    Assume carla_actors is a list of carla actors controlled by IAI
    """
    for agent_id, agent in enumerate(response.agent_states):
        if is_iai[agent_id]:
            agent_transform = transform_iai_to_carla(agent)
            try:
                actor = iai_to_carla_mapping[agent_id]
                actor.set_transform(agent_transform)
            except:
                pass

    # carla_lights = get_carla_traffic_lights(world)
    # set_traffic_lights(response.traffic_lights_states,carla_lights)

    world.tick()

def initialize_simulation(args, world, agent_states=None, agent_properties=None):

    iai_seed = args.seed if args.seed is not None else random.randint(1,10000)
    traffic_lights_states = initialize_tl_states(world)

    #################################################################################################
    # Initialize IAI Agents
    map_center = args.map_center
    print(f"Call location info.")
    location_info_response = iai.location_info(
        location = args.location,
        rendering_fov = args.fov,
        rendering_center = map_center
    )
    print(f"Begin initialization.") 
    # Acquire a grid of 100x100m regions in which to initialize vehicles to be controlled by IAI.
    regions = iai.get_regions_default(
        location = args.location,
        total_num_agents = args.num_agents,
        area_shape = (int(args.width/2),int(args.height/2)),
        map_center = map_center, 
    )
    # Place vehicles within the specified regions which will consider the relative states of nearby vehicles in neighbouring regions.
    response = iai.large_initialize(
        location = args.location,
        regions = regions,
        traffic_light_state_history = [traffic_lights_states],
        agent_states = agent_states,
        agent_properties = agent_properties,
        random_seed = iai_seed
    )

    # print(location_info_response)

    return response

