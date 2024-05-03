from BRT_Factory import conservative_BRT, predictive_BRT
from rl_agents.agents.common.factory import agent_factory

def get_agent(agent_choice):
    if agent_choice == "planner_agent":
        agent_config = {
            "__class__": "<class 'rl_agents.agents.tree_search.deterministic.DeterministicPlannerAgent'>",
            "env_preprocessors": [{"method":"simplify"}],
            "budget": 50,
            "gamma": 0.7,
        }
    return agent_factory(env, agent_config)

def predictive_BRT():
    pass

def conservative_BRT(env):
    # Test BRTCalculator  
    from highway_env.envs.common.safety import BRTCalculator
    conservative_BRT = BRTCalculator(env, conservative = True)
    return conservative_BRT

def get_BRT(env, type_brt):
    if type_brt == "conservative":
        return conservative_BRT(env)
    if type_brt == "predictive":
        return predictive_BRT(env)
    
def safety_wrap_action(user_choice, brt, agent, obs):
    if user_choice == "conservative":
        # decide when to use safe action, when to use nominal control 
        return agent.act(obs)
    return agent.act(obs)