##
#%%
import gym
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

##
#%%

class EnvNode(Node):
    def __init__(self,env):
        super().__init__("env_node")
        self.get_logger().info("Environment node test has been started")
        self.env = env

        ## PUBLISHER
        self.obs_publisher = self.create_publisher(
            Float64MultiArray, "obs/topic/array",10
        )#Receive from the ai node

        # SUBSCRIBER
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, "ai_node/topic/twist",self.cmd_vel_callback,10
        )#Send to the ai node 

        # Env 
        self.run_initialisation()
        
    def run_initialisation(self):
        self.get_logger().info("Starting of the environment initialisation")
        obs = self.env.reset()
        self.publish_obs(obs)
        self.get_logger().info("End of initialisation")

    def cmd_vel_to_action(self,cmd_vel : Twist) -> list:
        """From a Twist type return an action type that depend of the environment"""
        return [cmd_vel.angular.x, cmd_vel.angular.y]

    def obs_to_array(self,obs : list) -> Float64MultiArray:
        """From an observation type that depend of the environment return an Odometry type"""
        array = Float64MultiArray()
        array.data = list(
            map(
            lambda ob : ob[0] if type(ob) == list else 0,
            obs
            ))
        return array

    def publish_obs(self,obs : list) -> None:
        """Write on the topic that the ai node is subscribed"""
        array = self.obs_to_array(obs)
        self.get_logger().info(f"Try to publish observation : {str(array.data)}")
        self.obs_publisher.publish(array)
        self.get_logger().info(f"Obersvation publish : done")

    def cmd_vel_callback(self,cmd_vel : Twist) -> None:
        """When a callback is called, this test node have to send the new observation"""
        action = self.cmd_vel_to_action(cmd_vel)
        self.get_logger().info(f"Action received : {str(action)}")
        obs,reward ,done,info = self.env.step(action)
        self.get_logger().info(f"New step : observation {str(obs)} | done {str(done)} ")
        if done: 
            self.get_logger().info("End of the environment : well done")
            return
        self.publish_obs(obs)

##
#%%
class TestEnv:

    actions_path = "/home/smaug/Documents/ProjetGate-VoitureAutonome/ros2_ws/src/hl_autotech/liste_actions.txt"
    observations_path = "/home/smaug/Documents/ProjetGate-VoitureAutonome/ros2_ws/src/hl_autotech/liste_observations.txt"
    
    @staticmethod
    def equal(array1,array2):
        for a1,a2 in zip(array1,array2):
            if abs(a2-a1) > 1e-5:
                return False
        return True

    def __init__(self):

        actions = open(TestEnv.actions_path, "r").read().split("\n")
        observations = open(TestEnv.observations_path, "r").read().split("\n")
        self.observations = list(map(
            eval,
            filter(lambda s : len(s) > 2, observations)
            ))
        self.actions = list(map(
            eval,
            filter(lambda s : len(s) > 2, actions)
            ))
        pass

    def reset(self):
        self.i = 0
        return self.observations[0]

    def step(self,action):
        if not TestEnv.equal(action,self.actions[self.i]): 
            raise ZeroDivisionError()
        self.i+=1
        if self.i>=len(self.observations):
            return [None,None,True,None]
        return [self.observations[self.i],None,False,None]

env = TestEnv()




##
#%%
import gym
import highway_env
from stable_baselines3 import PPO

def main(args=None):
    env = gym.make("rnd_course-v0")
    rclpy.init(args=args)
    node = EnvNode(env)
    rclpy.spin(node)
    rclpy.shutdown()

# %%
 