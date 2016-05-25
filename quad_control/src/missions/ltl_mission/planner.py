import pickle
import numpy as np
import networkx as nx

class MonotonePlanner():
    def __init__(self,monotone_system,fts,env,verbose=0):
        self.fts = fts
        self.monotone_system = monotone_system
        self.env = env
        self.period = monotone_system.period
        self.last_time = 0.0
        self.fts_state = None
        self.control = None
        self.speed_controller = None

        self.position = None

        self.execute_plan = False
        
        self.control = None

        self.verbose = verbose
        self.name = "planner"

    def init_state(self,position):
        cell = self.monotone_system.get_cell(position)
        for n in self.fts.graph['initial']:
            if cell==n[0]:
                self.fts_state = n
                break

        self.control = nx.get_node_attributes(self.fts,"apply_control")

        return self.control[self.fts_state]

    def get_next_state(self,position):
        cell = self.monotone_system.get_cell(position)
        for n in self.fts.successors(self.fts_state):
            if cell==n[0]:
                return n

    def transition_available(self,sim):
        return sim.time-self.last_time>=self.period

    def di_sim(self,position):
        prev_fts_state = self.fts_state
        self.fts_state = self.get_next_state(position)
        if self.fts_state:
            return self.control[self.fts_state]
        else:
            s = "No fts state!!\n"
            s+= "------------------------\n"
            s+= "Current continuous state:" + str(position)+"\n"
            s+= "Current observation:" + str(self.monotone_system.get_cell(position)) + "\n"

            s+= "Previous state of the plan:"+ str(prev_fts_state)+ "\n"
            s+= "Possible transitions:" + "\n"
            cell = self.monotone_system.get_cell(position)
            for n in self.fts.successors(prev_fts_state):
                s+= str(n) + "\n"
            s+= "------------------------\n"
            return s
