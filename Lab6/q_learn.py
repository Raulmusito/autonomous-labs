from matplotlib import pyplot as plt
import numpy as np
from map import MapEnv
import tqdm
import copy



class QLearning:
    def __init__(self, env, alpha, gamma, epsilon, n_episodes,visualize):
        self.avg_reward = 0
        self.env = env
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.n_episodes = n_episodes 
        self.Q = np.random.rand(env.map.shape[0], env.map.shape[1], env.actions)

    def epsilon_greedy_policy(self, s, epsilon):
        # Epsilon greedy policy (choose a random action with probability epsilon)
        a = None
        r = np.random.rand()
        if r<=epsilon: a = np.random.choice(range(self.env.actions))
        else: a = np.argmax(self.Q[s]) #a = np.max(self.Q)
        return a
    
    def episode(self, alpha, epsilon,render=False):
        # Episode execution. Generate an action with epsilon_greedy_policy, call step, appy learning function
        # This function should return the total reward obtained in this episode
        # ...
        a = None
        episodic_reward = 0
        self.env.current_state= self.env.reset()
        prev_state = self.env.get_state()
        for step in range(self.env.max_steps):
            a = self.epsilon_greedy_policy(prev_state,epsilon)
            new_state,reward,ended = self.env.step(a)
            self.Q[prev_state[0],prev_state[1],a] = self.Q[prev_state[0],prev_state[1],a] + \
            alpha * (reward + self.gamma * np.max(self.Q[new_state[0],new_state[1],:]) - self.Q[prev_state[0],prev_state[1],a])

            # self.Q[prev_state[0],prev_state[1],a] =  self.Q[prev_state[0],prev_state[1],a] + \
            #     alpha*(reward+self.gamma*np.max(self.Q[new_state[0],new_state[1],:]))-self.Q[prev_state[0],prev_state[1],a]
            # print("prev_state",prev_state,"action:",a," next_state:",new_state)
            prev_state = new_state
            episodic_reward+=reward
            # print("epi_reward:",episodic_rewar
            if render:
                self.env.render_live(episodic_reward=episodic_reward,avg_reward=self.avg_reward)
            if ended:
                break
        
        return episodic_reward

    def train(self, n_episodes, check_every_n=100,render=False):
        """Execute n_episodes and every `check_every_n` episodes print the average reward and store it.
           As the initial position is random, this number will not be super stable..."""
        reward = []
        for eps in tqdm.tqdm(range(n_episodes)):
            r = self.episode(self.alpha,self.epsilon,render)
            if eps%check_every_n==0:
                reward.append(r)
                self.avg_reward=sum(reward)/len(reward)
                # print("Average Reward:",sum(reward)/len(reward))
        print("Average Reward:",sum(reward)/len(reward))

    def test_episode(self,policy,render=False):
                # Episode execution. Generate an action with epsilon_greedy_policy, call step, appy learning function
                # This function should return the total reward obtained in this episode
                # ...
                a = None
                episodic_reward = 0
                self.env.current_state= self.env.reset()
                prev_state = self.env.get_state()
                for step in range(self.env.max_steps): 
                    a = policy[prev_state]
                    new_state,reward,ended = self.env.step(a)
                    prev_state = new_state
                    episodic_reward+=reward
                    # print("epi_reward:",episodic_reward)
                    self.env.render_live(episodic_reward=episodic_reward,avg_reward=None)
                    if list(new_state) ==  all(self.env.goal):
                         print("Goal Reached\n")
                    if ended:
                        break
                plt.pause(2)

    def get_optimal_policy(self):
        """Once training is done, retrieve the optimal policy from Q(s,a)."""
        policy = np.zeros(self.env.map.shape, dtype=int)
        states = np.argwhere(self.env.map == 0)
        for s in states:
            opt_a = np.argmax(self.Q[s[0],s[1],:]) #in a particlar state check for best action
            policy[s[0],s[1]]=opt_a
        return policy

    def value_function(self):
        """Once training is done, retrieve the optimal value function from from Q(s,a)"""
        v = np.max(self.Q, axis = 2)
        v[self.env.map == 1] = -np.inf
        return v
 

# Proposed Map
map=[
[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
[1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1],
[1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1],
[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1],
[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1],
[1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1],
[1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
[1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
[1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
[1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1],
[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1],
[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1],
[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]


# Convert Map 2D array to Numpy array
grid_map = np.array(map)

# Show grid map
plt.matshow(grid_map, cmap = "jet")
plt.title('Base Map')
plt.colorbar()

print(np.shape(grid_map))
# Create an environment and a QLearning agent to learn it. Plot the averaged rewards stored.
eps=5000
eps_steps =30
env = MapEnv(map=grid_map, goal=np.array([4, 4]), max_steps=eps_steps)
agent = QLearning(env, alpha=0.003, gamma=0.95, epsilon=0.5, n_episodes=eps,visualize=False)
agent.train(n_episodes=eps, check_every_n=eps/4,render=False)
optimal_policy = agent.get_optimal_policy()
optimal_value = agent.value_function()


# Plot the value function (see included figure)
plt.subplot(1, 2, 1)
plt.imshow(optimal_value, cmap="viridis", interpolation="nearest")
plt.colorbar(label="Value (V(s))")
plt.title("Value Function")
plt.xlabel("State (Column)")
plt.ylabel("State (Row)")

# Plot policsy (see included figure)
plt.subplot(1, 2, 2)
plt.imshow(optimal_policy, cmap="viridis", interpolation="nearest")
plt.colorbar(label="Action")
plt.title("Optimal Policy")
plt.xlabel("State (Column)")
plt.ylabel("State (Row)")

plt.tight_layout()
plt.show()


agent.test_episode(optimal_policy,render=True)


