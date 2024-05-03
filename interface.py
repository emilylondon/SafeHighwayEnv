# from highway_planning.ipynb
#@title Imports for env, agent, and visualisation.

# Environment
import gymnasium as gym

# Safety wrapper demo
from safety_wrapper import safety_wrap_action, get_BRT, get_agent, process_safety_results, display_results

# Visualisation
import sys
from tqdm.notebook import trange
sys.path.insert(0, './highway-env/scripts/')
from scripts.utils import record_videos, show_videos

# User choices
choices = {
    "BRT": ["conservative"],
    "Agent": "planner_agent",
}

# Make environment
env = gym.make("intersection-v0", render_mode="rgb_array")
env = record_videos(env)
(obs, info), done = env.reset(), False

# Make agent
agent = get_agent(choices["Agent"])

# Provide agent 
# agent = provide_agent(my_agent)

# Provide predictor 
# Y = load_my_predictor()

# List of BRTs 
BRTs = [(user_choice, get_BRT(env, user_choice)) for user_choice in choices["BRT"]]

# User Prediction BRT 
# BRTs = [(user_choice, get_BRT(env, user_choice, Y)) for user_choice in choices["BRT"]]

# No BRT
BRTs.append[("None", None)]

# User specified additional params
# additional_params = {} 

# Run episode
results = {}
for (user_choice, brt) in BRTs:
    results[user_choice] = {}
    for step in trange(env.unwrapped.config["duration"], desc="Running..."):
        action = safety_wrap_action(user_choice, brt, agent, obs)
        # action = user_safety_wrap_action(agent, obs, additional_params)
        obs, reward, done, truncated, info = env.step(action)
        results[user_choice][step] = process_safety_results(BRTs, obs, reward, info, action, step)
        
    env.close()
    show_videos()
display_results(results)
