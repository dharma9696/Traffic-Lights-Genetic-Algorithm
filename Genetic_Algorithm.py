# -*- coding: utf-8 -*-
"""
Created on Fri Mar 25 03:39:29 2022

@author: rohit
"""

import os
import sys
from  warnings import filterwarnings
import numpy as np
from time import sleep


from  Simulation_Generator import get_options
filterwarnings("ignore")

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
    
    
import traci 
from sumolib import checkBinary

#Sumo variables
sumo_binary = checkBinary('sumo')
sumo_binary_gui = checkBinary('sumo-gui')
config_file = 'Manhattan5x3.sumocfg'
n_steps = 1000
visualize_delay = 100 #delay in ms

#Genetic Algorithm parameters
pop_size = 15
max_generations = 100  
n_survivors = 3

#Mutation and Crossover parameters
crossover_rate=0.5
duration_mutation_rate=0.2
duration_mutation_strength=5
states_mutation_rate = 0.2
light_options = ['G', 'y', 'r']


#Fitness Function parameters
collision_penalty = 400
waiting_time_weight = 1/2000
emissions_weight = 1/2000





class TLight(object):
    def __init__(self,ID):
        '''
        Initialize a new data storing object to keep track of the
        emissions and waiting time data over time
        '''

        self.ID = ID
        self.durations = []
        self.states = []

    def get_state_duration(self,ID):
        states=[]
        durations=[]
        definitions = traci.trafficlight.getCompleteRedYellowGreenDefinition(ID)
        for definition in definitions: 
            for phase in definition.phases:
                states.append(phase.state)
                durations.append(phase.minDur)
        
        self.states = states
        self.durations = durations
        return states,durations
    
    def set_state_duration(self,ID,states,durations):
        self.ID = ID
        self.states = states
        self.durations = durations
        definition = traci.trafficlight.getCompleteRedYellowGreenDefinition(ID)[0]
        idx = 0
        for phase in definition.phases:
            # print(ID,states,durations)
            # set the current phase's durations and state
            phase.minDur = phase.maxDur = phase.duration = durations[idx]
            phase.states = states[idx]
            idx += 1
            
        logic = traci.trafficlight.Logic(traci.trafficlight.getProgram(ID), 0, 0, phases=definition.phases)
        # print(ID,logic)
        traci.trafficlight.setCompleteRedYellowGreenDefinition(ID, logic)
        
        return self



class population(object):
    def __init__(self,generation):
        self.generation = generation
        self.gene_pool = []
        self.best_individual= None
        self.best_fitness = None
        self.best_emissions = None
        self.best_waiting_time = None
        self.avg_fitness = None
        self.avg_waiting_time = None
        self.avg_emissions = None
        self.json = None


    def print_pop(self):
        for individual,chromosome in zip(range(len(self.gene_pool)),self.gene_pool):
            print('--------------------------------------')
            print('--------------------------------------')
            print('--------------------------------------\n')
            print("Generation :",self.generation)
            print("Individual :",individual)
            print_chromosome(chromosome)
            
    def evaluate_pop(self):
        fitness_pop= []
        emissions_pop= []
        waiting_pop= []
        individual=0
        for chromosome in self.gene_pool:
            print("Generation:",self.generation," Individual: ",individual)
            fitness, emissions, waiting_time = evaluate_chromosome(chromosome)
            fitness_pop.append(fitness)
            emissions_pop.append(emissions)
            waiting_pop.append(waiting_time)
            individual=individual+1
        
        self.best_fitness = np.min(fitness_pop)
        self.best_emissions = np.min(emissions_pop)
        self.best_waiting_time = np.min(waiting_pop)
        self.avg_fitness = np.mean(fitness_pop)
        self.avg_emissions = np.mean(emissions_pop)
        self.avg_waiting_time = np.mean(waiting_pop)
        self.best_individual= self.gene_pool[np.argmin(fitness_pop)]
        
        return fitness_pop,emissions_pop,waiting_pop

def get_chromosome():
    
    traci.start([sumo_binary, "-c", config_file,'--start','--quit-on-end'],label = 'sim2')
    print("SUMO launched: Collecting Traffic Lights")
    conn = traci.getConnection("sim2")
    print(conn)    
    TLightIDs = conn.trafficlight.getIDList()
    Chromosome=[]
    for Junction in TLightIDs:
        states = []
        durations=[]
        TL1 = TLight(Junction)
        states,durations = TL1.get_state_duration(Junction)
        Chromosome.append(TL1) 
        
    traci.close()
    return Chromosome



def set_chromosome(chromosome):

    new_chromosome = []
    for Junction in chromosome:
        # updated_TL1 = TLight(Junction)
        ID = Junction.ID
        states = Junction.states
        durations=Junction.durations
        TL1 = TLight(Junction)
        updated_TL1 = TL1.set_state_duration(ID,states,durations)
        new_chromosome.append(updated_TL1)
    
    return new_chromosome


def print_chromosome(chromosome):
    for Junction in chromosome:
        ID = Junction.ID
        states = Junction.states
        durations=Junction.durations
        print('-----------------------------------')
        print("Signal ID :",ID,"\n")
        print("States \t \t Durations")
        print('-----------------------------------')
        for states,durations in zip(states,durations):
            print(states,"\t \t", durations)
           



def mutate_chromosome(chromosome,duration_mutation_rate=duration_mutation_rate,
                      duration_mutation_strength = duration_mutation_strength,
                      states_mutation_rate = states_mutation_rate ):

    chromosome_new = []
    for Tlight in chromosome:
        ID = Tlight.ID
        Tlight_new =TLight(ID)
        durations = Tlight.durations
        states = Tlight.states
        durations_new = []
        states_new = []
        # np.random.seed(seed=int(time.time())) 
        RNG = (np.random.uniform(0,1,len(durations)) <= duration_mutation_rate)*1

        for D1,i in zip(durations,range(len(RNG))):
            D1 = max(round(D1 + RNG[i]* np.random.normal(0,duration_mutation_strength),1),1)
            durations_new.append(D1)
        
        for S1,i in zip(states,range(len(RNG))):
            lane_colors = list(S1)
            lane_colors_new = []
            for c1 in lane_colors:
                # np.random.seed(seed=int(time.time())) 
                if np.random.uniform(0,1) < states_mutation_rate:
                    lane_colors_new.append(np.random.choice(light_options))
                else:
                    lane_colors_new.append(c1)
                    
            lane_colors_new = ''.join(lane_colors_new)
            states_new.append(lane_colors_new)
            
        #     states_new.append(lane_colors_new)
        # print(states_new)    
            
        Tlight_new.durations = durations_new
        Tlight_new.states = states_new
        chromosome_new.append(Tlight_new)
    
    return chromosome_new




def crossover_parent(chromosome_male,chromosome_female,
                     crossover_rate = crossover_rate):
    chromosome_child = []
    # for Tlight_male,Tlight_female in zip(chromosome_male,chromosome_female):
    for Tlight in range(len(chromosome_male)):
        
        ID = chromosome_male[Tlight].ID
        Tlight_child = TLight(ID)
        durations_male= chromosome_male[Tlight].durations
        states_male = chromosome_male[Tlight].states
        durations_female= chromosome_female[Tlight].durations
        states_female = chromosome_female[Tlight].states
        durations_child = []
        states_child = []
        # print(ID)
        
        for D_male,D_female in zip(durations_male,durations_female):
             # np.random.seed(seed=int(time.time())) 
             RNG = np.random.uniform(0,1)
             # print(rndm)
             durations_child.append(D_male if RNG < crossover_rate else D_female)
             
             
        for S_male,S_female in  zip(states_male,states_female):
            lane_colors_male = list(S_male)
            lane_colors_female = list(S_female)
            lane_colors_child=[]
            # for c_male,c_female in zip(lane_colors_male,lane_colors_female):
            for i in range(len(lane_colors_male)):
                
            # print("\t\t",c_male,c_female)
                # np.random.seed(seed=int(time.time())) 
                RNG= np.random.uniform(0,1)
                if RNG < crossover_rate:    
                    lane_colors_child.append(lane_colors_male[i])
                else:
                    lane_colors_child.append(lane_colors_female[i])
                # print("\t\t",lane_colors_child)
            lane_colors_child = ''.join(lane_colors_child)
            states_child.append(lane_colors_child)
            
        Tlight_child.durations = durations_child
        Tlight_child.states = states_child
        chromosome_child.append(Tlight_child)

    
    return chromosome_child
    




def evaluate_chromosome(chromosome,sumo_binary=sumo_binary,
                        config_file = config_file,n_steps = n_steps):
    # instantiate a new SUMO instance and set insert the current genome
    # l1 = Logic(programID='0', type=0, currentPhaseIndex=0, 
    #       phases=(Phase(duration=89.6, state='GGGGGG', minDur=89.6, maxDur=89.6, next=()), 
    #               Phase(duration=1, state='yyyyyy', minDur=1, maxDur=1, next=()), 
    #               Phase(duration=471.8, state='GGGGGG', minDur=471.8, maxDur=471.8, next=()), 
    #               Phase(duration=5.3, state='yyyyyy', minDur=5.3, maxDur=5.3, next=())), 
    #       subParameter={})

    traci.start([sumo_binary, "-c", config_file,'--start','--quit-on-end'],label = 'sim2')
    print("SUMO launched")
    conn = traci.getConnection("sim2")
    print(conn)
    
    

    lanes  =  conn.lane.getIDList()
    set_chromosome(chromosome)
    # print("Updated Chromosome")
    # print_chromosome(get_chromosome())

    lanes  =  conn.lane.getIDList()
    lane_emissions = np.zeros((n_steps+1,len(lanes)))
    lane_waiting = np.zeros((n_steps+1,len(lanes)))
    fitness = 0
    for step in range(n_steps):
        # take one step in the simulation
        traci.simulationStep()
        # check for collisions in the current time step
        if traci.simulation.getCollidingVehiclesNumber() > 0:
            # break the current simulation and penalize the genome's fitness
            fitness += collision_penalty
            break

        for l1,i in zip(lanes,range(len(lanes))):
    # compute per lane scores
            lane_emissions[n_steps][i] += traci.lane.getCO2Emission(l1)
            lane_waiting[n_steps][i] += traci.lane.getWaitingTime(l1)

    # compute the total fitness score for the current genome
        fitness += np.sum(np.sum(lane_emissions)) + np.sum(np.sum(lane_waiting))
    # close the simulation instance and return the fitness
    traci.close()
    #data.store(emissions, waiting)
    return int(fitness), int(np.sum(np.sum(lane_emissions))), int(np.sum(np.sum(lane_waiting)))


def generate_random_population(chromosome,pop_size = pop_size):

    init_pop  =  population(0)
    gene_pool = []
    for individual in range(pop_size):
        chromosome_new = mutate_chromosome(chromosome,1,10,1)
        chromosome = chromosome_new
        gene_pool.append(chromosome_new)
    
    init_pop.gene_pool = gene_pool
    
    return init_pop



def visualize_SUMO(chromosome=None,n_steps=n_steps):
    traci.start([sumo_binary_gui, "-c", config_file,'--start','--quit-on-end'],label = 'simView')
    if chromosome is not None:
        set_chromosome(chromosome)
    
    for step in range(n_steps):
        # take one step in the simulation
        traci.simulationStep()
        sleep(0.1)
    traci.close()
    


def run_GA(max_generations = max_generations,n_survivors = n_survivors):
    # traci.start([sumo_binary, "-c", config_file,'--start','--quit-on-end'],label = 'sim1')
    # conn = traci.getConnection("sim1")

    chromosome_base = get_chromosome()
    Generations = []
    GA_pop_next = generate_random_population(chromosome_base)
    current_gen = 0
    while(current_gen <max_generations):
        GA_pop = GA_pop_next
        
        fitness, emissions, waiting = GA_pop.evaluate_pop()
        Generations.append([GA_pop.generation,GA_pop.best_fitness,GA_pop.best_emissions,GA_pop.best_waiting_time,
                            GA_pop.avg_fitness,GA_pop.avg_emissions,GA_pop.avg_waiting_time])
        
        print("Best Fitness value in Generation ",current_gen, " is ",GA_pop.best_fitness)
        
        if current_gen // 5 == 0:
            visualize_SUMO(GA_pop.best_individual)

        sorted_pop = np.argsort(fitness)[:n_survivors]
        Next_gene_pool = [GA_pop.gene_pool[idx] for idx in sorted_pop]
        while len(Next_gene_pool) < pop_size:
    
            chromosome_male =GA_pop.gene_pool[int(np.random.uniform(0,n_survivors))]
            chromosome_female=GA_pop.gene_pool[int(np.random.uniform(0,n_survivors))]
            chromosome_child = crossover_parent(chromosome_male, chromosome_female)
            chromosome_child = mutate_chromosome(chromosome_child,0.1,5,0.2)
            Next_gene_pool.append(chromosome_child)
        
        current_gen = current_gen+1
        GA_pop_next = population(current_gen)
        GA_pop_next.gene_pool = Next_gene_pool
        
    return GA_pop,Generations






# if __name__ == "__main__":
#     options = get_options()

#     # this script has been called from the command line. It will start sumo as a
#     # server, then connect and run
#     if options.nogui:
#         sumoBinary = checkBinary('sumo')
#     else:
#         sumoBinary = checkBinary('sumo-gui')


# chromosome_base = get_chromosome()
# GA_pop_next = generate_random_population(chromosome_base)

# GA_pop_next.print_pop()


# Pop_dict={}
# Pop_dict['generation'] = GA_pop_next.generation
# Pop_dict['best_individual'] = GA_pop_next.best_individual
# Pop_dict['best_fitness'] = GA_pop_next.best_fitness
# Pop_dict['best_emissions'] = GA_pop_next.best_emissions
# Pop_dict['best_waiting_time'] = GA_pop_next.best_waiting_time
# Pop_dict['avg_fitness'] = GA_pop_next.avg_fitness
# Pop_dict['avg_waiting_time'] = GA_pop_next.avg_waiting_time
# Pop_dict['avg_emissions'] = GA_pop_next.avg_emissions



# self.generation = generation
# self.gene_pool = []
# self.best_individual= None
# self.best_fitness = None
# self.best_emissions = None
# self. = None
# self.avg_fitness = None
# self.avg_waiting_time = None
# self.avg_emissions = None
# run_GA()


