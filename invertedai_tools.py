import invertedai as iai
from invertedai.common import AgentProperties, AgentState, TrafficLightState
import carla
import random
import math

def setup_carla_environment(args):
    step_length = 0.1 #0.1 is the only step length that is supported at this time

    client = carla.Client(args.host, args.port)
    client.set_timeout(200.0)

    # Configure the simulation environment
    #world = client.load_world(args.location.split(":")[-1])
    world = client.get_world()
    world_settings = carla.WorldSettings(
        synchronous_mode=True,
        fixed_delta_seconds=step_length,
    )
    world.apply_settings(world_settings)

    return client, world

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

def transform_iai_to_carla(agent_state):
    agent_transform = carla.Transform(
        carla.Location(
            agent_state.center.x,
            agent_state.center.y,
            0.
        ),
        carla.Rotation(
            yaw=math.degrees(agent_state.orientation)
        )
    )

    return agent_transform

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

    world.tick()

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

            blueprint = random.choice(vehicle_blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            agent_transform = transform_iai_to_carla(state)

            actor = world.try_spawn_actor(blueprint,agent_transform)
            
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

def get_traffic_lights_mapping(world):
    tls = world.get_actors().filter('traffic.traffic_light*')
    tl_ids = sorted([tl.id for tl in list(tls)])
    carla2iai = {}
    # ID for IAI traffic lights, only valid for Town10 for now (in both UE4 and UE5 versions of the map)
    iai_tl_id = 4364
    for carla_tl_id in tl_ids:
        carla2iai[str(carla_tl_id)] = [str(iai_tl_id), str(iai_tl_id+1000)]
        iai_tl_id+=1

    return carla2iai

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

def set_traffic_lights_from_carla(iai_tl, carla_tl, carla2iai):
    # Assume carla_lights is a dictionary containing ID's of traffic light actors mapped to Carla traffic light agents
    for carla_tl_id, carla_state in carla_tl.items():
        iai_tl_id_pair = carla2iai[carla_tl_id]
        for iai_tl_id in iai_tl_id_pair:
            iai_tl[iai_tl_id] = get_traffic_light_state_from_carla(carla_state)
    return iai_tl

def assign_iai_traffic_lights_from_carla(world, iai_tl, carla2iai):

    traffic_lights = world.get_actors().filter('traffic.traffic_light*')
    
    carla_tl_dict = {}
    for tl in traffic_lights:
        carla_tl_dict[str(tl.id)]=tl.state

    return set_traffic_lights_from_carla(iai_tl, carla_tl_dict, carla2iai)

def initialize_tl_states(world):
    carla2iai = get_traffic_lights_mapping(world)
    iai_tl_states = {}
    for tlpair in carla2iai.values():
        for tl in tlpair:
            iai_tl_states[tl] = TrafficLightState.red # Initialize to given value

    iai_tl_states = assign_iai_traffic_lights_from_carla(world, iai_tl_states, carla2iai)
    return iai_tl_states, carla2iai

# Initialize InvertedAI co-simulation
def initialize_simulation(args, world, agent_states=None, agent_properties=None):

    iai_seed = args.seed if args.seed is not None else random.randint(1,10000)
    traffic_lights_states, carla2iai_tl = initialize_tl_states(world)

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
        total_num_agents = args.number_of_vehicles,
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

    return response, carla2iai_tl, location_info_response

