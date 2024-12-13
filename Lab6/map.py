from matplotlib import pyplot as plt
import numpy as np

class MapEnv:
    def __init__(self, map, goal, max_steps):
        self.map = map
        self.current_state = None
        self.goal = goal.astype(np.int32)
        self.actions = 4
        self.steps = 0
        self.max_steps = max_steps
        if map[goal[0], goal[1]] != 0:
            raise ValueError("Goal position is an obstacle")

    def reset(self):
        # start the agent in a random position within the map and return agent state (cell in which it is)
        self.steps = 0
        free_cells = np.argwhere(self.map == 0)
        if len(free_cells) == 0:
            raise ValueError("Map contains no free cells to place the agent.")
        r = free_cells[np.random.choice(len(free_cells))]

        self.current_state = tuple(r)
        return self.current_state
 

    def step(self, action):
      # this function applies the action taken and returns the obtained state, a reward and a boolean that says if the episode has ended (max steps or goal reached) or not (any other case)
      # action: 0 = up, 1 = down, 2 = left, 3 = right
      ended = False
      reward = -1
      a = [(-1, 0), (1, 0),(0, -1),(0, 1)]

      x,y = self.get_state()
      new_state = self.current_state
      dx,dy = a[action]
      if 0 <= x+dx < self.map.shape[0] and 0 <= y+dy < self.map.shape[1] and self.map[x+dx,y+dy]!=1:
        
        new_state = x+dx,y+dy
        
        if new_state== tuple(self.goal): 
          reward = 1
          ended = True
        else: reward =-1

      if self.steps == self.max_steps:
        ended =True 
          
      self.current_state = new_state
      self.steps+=1

      return new_state,reward,ended

    def get_state(self):
      # returns current state
      return self.current_state

    def render(self, i=0):
        plt.clf()
        plt.matshow(self.map, cmap = "jet")
        plt.title('Map')
        plt.colorbar()
        plt.scatter(self.current_state[1], self.current_state[0], c = 'r')
        plt.scatter(self.goal[1], self.goal[0], c = 'g')
        # plt.savefig("q_learning_{0:04}.png".format(i), dpi = 300)
        # plt.show()
        plt.pause(0.1) 

    def render_live(self, i=0, episodic_reward=0,avg_reward=0,save_frames=False):
        if not hasattr(self, 'fig'):   
            self.fig, self.ax = plt.subplots()  
            self.im = self.ax.imshow(self.map, cmap="jet")   
            self.ax.scatter(self.current_state[1], self.current_state[0], c='r' )  # Agent
            self.ax.scatter(self.goal[1], self.goal[0], c='g' )  # Goal
            # self.ax.legend()
            # self.fig.colorbar(self.im)
        else:
            self.ax.clear()  # Clear the previous plot
            self.ax.imshow(self.map, cmap="jet")  # Redraw the map
            self.ax.scatter(self.current_state[1], self.current_state[0], c='r' )  # Update agent
            self.ax.scatter(self.goal[1], self.goal[0], c='g')   # Update goal
            # self.ax.legend()

        if episodic_reward is not None:
            self.ax.text(
                0.05, 0.98, f"Episodic Reward: {int(episodic_reward)}", 
                transform=self.ax.transAxes, fontsize=10, color="white", 
                verticalalignment="top", horizontalalignment="left", 
                fontweight='bold' 
                
            )
        if avg_reward is not None:
            self.ax.text(
                0.65, 0.98, f"Average Reward: {int(avg_reward)}", 
                transform=self.ax.transAxes, fontsize=10, color="white", 
                verticalalignment="top", horizontalalignment="left", 
                fontweight='bold' 
            )
        episodic_reward

        if save_frames:
            self.fig.savefig(f"q_learning_{i:04}.png", dpi=300)  # Save frames

        plt.pause(0.1)  