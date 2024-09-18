import invertedai as iai
from invertedai.common import AgentAttributes, AgentState, StaticMapActor
import carla

import argparse
from tqdm import tqdm
import matplotlib.pyplot as plt
import time
import random
import math

iai.add_apikey('')  # specify your key here or through the IAI_API_KEY variable

z_offset = 0.15

def setup_carla_environment(args):
    step_length = 0.1 #0.1 is the only step length that is supported at this time

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)

    # Configure the simulation environment
    world = client.load_world(args.location.split(":")[-1])
    world_settings = carla.WorldSettings(
        synchronous_mode=True,
        fixed_delta_seconds=step_length,
    )
    world.apply_settings(world_settings)

    return client, world

def get_vehicle_blueprint_list(world):
    # A method that returns a list of Carla vehicle blueprints based on any desireable criteria such as vehicle class, size, etc.

    # REPLACE THIS LIST WITH A LIST OF DESIRED VEHICLE BLUEPRINTS
    blueprint_list = [
        world.get_blueprint_library().find(bp_str) for bp_str in [
            "vehicle.audi.a2",
            "vehicle.audi.etron",
            "vehicle.audi.tt",
            "vehicle.bmw.grandtourer",
            "vehicle.citroen.c3",
            "vehicle.chevrolet.impala",
            "vehicle.dodge.charger_2020",
            "vehicle.ford.mustang",
            "vehicle.ford.crown",
            "vehicle.jeep.wrangler_rubicon",
            "vehicle.lincoln.mkz_2020",
            "vehicle.mercedes.coupe_2020",
            "vehicle.nissan.micra",
            "vehicle.nissan.patrol_2021",
            "vehicle.seat.leon",
            "vehicle.toyota.prius",
            "vehicle.volkswagen.t2_2021",
            # "vehicle.tesla.cybertruck",
            "vehicle.tesla.model3",
            "vehicle.mini.cooper_s"
        ]
    ]

    return blueprint_list

# Get a dictionary containing the length and width of the bounding box for each vehicle blueprint
def get_blueprint_dictionary(world, client):

    spawn_points = world.get_map().get_spawn_points()

    # Replace this list with the required blueprint filters, e.g., *audi* for audi vehicles, or "vehicle.audi.etron" for an specific model
    blueprint_filters = ["vehicle*"]
    # blueprint_filters = get_vehicle_blueprint_list(world)

    blueprint_list = []
    for bp_filter in blueprint_filters:
        print(bp_filter)
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

def assign_carla_blueprints_to_iai_agents(world,vehicle_blueprints,agent_properties,agent_states,recurrent_states):
    # A method to assign existing IAI agents to a Carla vehicle blueprint and add this agent to the Carla simulation
    # Return a dictionary of IAI agent ID's mapped to references to Carla agents within the Carla environment (e.g. actor ID's)

    iai_to_carla_mapping = {}
    agent_properties_new = []
    agent_states_new = []
    recurrent_states_new = []
    new_agent_id = 0
    for agent_id, (state, attr) in enumerate(zip(agent_states,agent_properties)):
        #blueprint = match_carla_actor(attr,vehicle_blueprints)
        blueprint = random.choice(vehicle_blueprints)
        agent_transform = transform_iai_to_carla(state)

        if agent_id==0: blueprint.set_attribute('role_name', 'hero')

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

            actor.set_simulate_physics(False)

    if len(agent_properties_new) == 0:
        raise Exception("No vehicles could be placed in Carla environment.")

    return iai_to_carla_mapping, agent_properties_new, agent_states_new, recurrent_states_new

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

def set_traffic_lights(traffic_lights_states,carla_lights):
    # Assume carla_lights is a dictionary containing ID's of traffic light actors mapped to Carla traffic light agents
    # This function is only a suggestion about how to set traffic lights
    for tl_id, state in traffic_lights_states.items():
        # carla_tl_id = get_carla_tl_id(tl_id)
        carla_tl_id = tl_id
        carla_lights[carla_tl_id].set_light(get_traffic_light_state_from_iai(state))

def carla_tick(iai_to_carla_mapping,response,world):
    """
    Tick the carla simulation forward one time step
    Assume carla_actors is a list of carla actors controlled by IAI
    """
    for agent_id, agent in enumerate(response.agent_states):
        agent_transform = transform_iai_to_carla(agent)

        actor = iai_to_carla_mapping[agent_id]
        actor.set_transform(agent_transform)

    # carla_lights = get_carla_traffic_lights(world)
    # set_traffic_lights(response.traffic_lights_states,carla_lights)

    world.tick()

def initialize_simulation(args):

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
        random_seed = random.randint(1,10000)
    )

    return response, location_info_response


def main(args):
    
    response, location_info_response = initialize_simulation(args)
    agent_properties = response.agent_properties
    map_center = args.map_center

    print(agent_properties)

    #################################################################################################
    # Setup Carla Environment
    print(f"Setting up Carla environment.")
    client, world = setup_carla_environment(args)

    #vehicle_blueprints = get_vehicle_blueprint_list(world)
    vehicle_blueprints = get_blueprint_dictionary(world, client)
    


    iai_to_carla_mapping, agent_properties, agent_states_new, recurrent_states_new = assign_carla_blueprints_to_iai_agents(world,vehicle_blueprints,agent_properties,response.agent_states,response.recurrent_states)
    response.agent_states = agent_states_new
    response.recurrent_states = recurrent_states_new
    carla_tick(iai_to_carla_mapping,response,world)

    #################################################################################################
    # Run simulation
    # Call drive over the large area to rollout all agents controlled by IAI forward one timestep then update these agents in Carla.

    print(f"Set up IAI visualization.")
    # Code for visualization purposes.
    if args.save_sim_gif:
        rendered_static_map = location_info_response.birdview_image.decode()
        scene_plotter = iai.utils.ScenePlotter(
            rendered_static_map,
            args.fov,
            map_center,
            [StaticMapActor.fromdict({
                "actor_id":actor.actor_id,
                "agent_type":actor.agent_type,
                "x":map_center[0]-actor.center.x,
                "y":actor.center.y,
                "orientation":math.pi-actor.orientation,
                "length":actor.length,
                "width":actor.width,
                "dependant":actor.dependant,
            }) for actor in location_info_response.static_actors],
            resolution=(2048,2048),
            dpi=300
        )
        scene_plotter.initialize_recording(
            agent_states=[AgentState.fromlist([
                map_center[0]-state.center.x,
                state.center.y,
                math.pi-state.orientation,
                state.speed
            ]) for state in response.agent_states],
            agent_properties=response.agent_properties,
            traffic_light_states=response.traffic_lights_states
        )

    print(f"Begin stepping through simulation.")
    for _ in tqdm(range(args.sim_length)):
        response = iai.large_drive(
            location = args.location,
            agent_states = response.agent_states,
            agent_properties = agent_properties,
            recurrent_states = response.recurrent_states,
            light_recurrent_states = response.light_recurrent_states,
            single_call_agent_limit = args.capacity,
            async_api_calls = args.is_async,
            random_seed = random.randint(1,10000)
        )

        carla_tick(iai_to_carla_mapping,response,world)

        if args.save_sim_gif: scene_plotter.record_step(
            [AgentState.fromlist([
                map_center[0]-state.center.x,
                state.center.y,
                math.pi-state.orientation,
                state.speed
            ]) for state in response.agent_states],
            response.traffic_lights_states
        )

    if args.save_sim_gif:
        print("Simulation finished, save visualization.")
        # save the visualization to disk
        fig, ax = plt.subplots(constrained_layout=True, figsize=(50, 50))
        plt.axis('off')
        current_time = int(time.time())
        gif_name = f'carla_demo_example_{current_time}_location-{args.location.split(":")[-1]}_density-{args.num_agents}_center-x{map_center[0]}y{map_center[1]}_width-{args.width}_height-{args.height}.gif'
        scene_plotter.animate_scene(
            output_name=gif_name,
            ax=ax,
            direction_vec=False,
            velocity_vec=False,
            plot_frame_number=True,
        )
    print("Done")


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)'
    )
    argparser.add_argument(
        '-p',
        '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)'
    )
    argparser.add_argument(
        '-N',
        '--num-agents',
        metavar='D',
        default=1,
        type=int,
        help='Number of vehicles to spawn in defined area in the map (default: 1)'
    )
    argparser.add_argument(
        '--sim-length',
        type=int,
        help="Length of the simulation in timesteps (default: 100)",
        default=100
    )
    argparser.add_argument(
        '--location',
        type=str,
        help=f"IAI formatted map on which to create simulate.",
        default='None'
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
    argparser.add_argument(
        '--save-sim-gif',
        type=bool,
        help=f"Should the simulation be saved with visualization tool.",
        default=True
    )
    args = argparser.parse_args()

    main(args)