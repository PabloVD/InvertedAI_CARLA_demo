import invertedai as iai
import os
import argparse

# Argument parser
def argument_parser():

    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--iai-key',
        type=str,
        help="InvertedAI API key."
    )
    argparser.add_argument(
        '--log-path',
        type=str,
        help="Path to the iai log file",
        default=os.path.join(os.getcwd(),f"iailog.json")
    )
    argparser.add_argument(
        '--gif-path',
        type=str,
        help="Path to the iai replaying gif file",
        default=os.path.join(os.getcwd(),f"scenario_log_example_replay.gif")
    )
    
    args = argparser.parse_args()

    return args

def main():

    args = argument_parser()

    iai.add_apikey(args.iai_key) 

    log_reader = iai.LogReader(args.log_path)
    gif_path_replay = args.gif_path
    
    log_reader.visualize(
        gif_path=gif_path_replay,
        fov = 200,
        resolution = (2048,2048),
        dpi = 300,
        map_center = None,
        direction_vec = False,
        velocity_vec = False,
        plot_frame_number = True
    )

main()