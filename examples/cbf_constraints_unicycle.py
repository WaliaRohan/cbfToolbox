# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
# 
# This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.
# 
# © 2023 Massachusetts Institute of Technology.
# 
# Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)
# 
# The software/firmware is provided to you on an As-Is basis
# 
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
#
# Author: Andrew Schoer, andrew.schoer@ll.mit.edu

import argparse
import numpy as np
import matplotlib.pyplot as plt
from cbf_toolbox.geometry import Sphere, Ellipsoid, HalfPlane
from cbf_toolbox.dynamics import Dynamics, UnicycleNID, SingleIntegrator2d
from cbf_toolbox.vertex import Agent, Obstacle, Goal
from cbf_toolbox.safety import Simulation

def main(P, Gamma, n, pred):

    save_file_name="p"+str(P)+"g"+str(Gamma)+"err"#+str(max_error_magnitude)

    a1 = Agent(state=np.array([-2.0,0.5]),
               shape=Sphere(0.5),
               dynamics=UnicycleNID(0, 0),
               plot_arrows=True,
               plot_path=True)

    # CBF constraint
    rightPlane = Obstacle(state=np.array([0.,10.]),
                  shape=HalfPlane(n=np.array([0,-1]), #n is the normal to the plane. It's sign defines the direction in which it is visible to an "agent"
                  rotation=0),
                  dynamics=SingleIntegrator2d(),
                  p = P) # "p" is the power of alpha in CBF constraint. It
                 # dictates how aggresive CBF is.

    leftPlane = Obstacle(state=np.array([0.,-10.0]),
                  shape=HalfPlane(n=np.array([0,1]), #n is the normal to the plane. It's sign defines the direction in which it is visible to an "agent"
                  rotation=0),
                  dynamics=SingleIntegrator2d(),
                  p = P) # "p" is the power of alpha in CBF constraint. It
                 # dictates how aggresive CBF is.
    
    # CLF Constraint
    g1 = Goal(np.array([5.,0.]), gamma=Gamma) # gamma = 0.25 is default
    
    # Now we can add everything to a simulation object
    s = Simulation()
    s.add_agent(agent=a1, control=g1)
    s.add_obstacle(obst=rightPlane)
    s.add_obstacle(obst=leftPlane)

    # When everything is added, we can call the simulate function
    # Before running the simulation, the function will loop over all the agents and obstacles and create
    # the proper Edges to connect the Vertex objects (the CBF and CLF objects)
    s.simulate(num_steps=n, dt = 0.1)#, video_name=save_file_name)

    s.plot_functions()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Parameters for simulation")

    parser.add_argument('--p', type=float, help='CBF argument (higher value -> stronger CBF), default: 1.0', default=1.0)
    parser.add_argument('--g', type=float, help='CLF argument (higher value -> strong CLF), default: 0.25', default=0.25)
    parser.add_argument('--n', type=int, help='Number of steps for simulation, default:100', default=100)
    parser.add_argument('--pred', type=bool, help='If true, generates control inputs with state estimation error, default: False', default=False)
    
    # parser.add_argument('--err', type=float, help='max error magnitude, default: 0', default=0.0)
    
    args = parser.parse_args()

    print("P: ", args.p)
    print("Gamma: ", args.g)
    print("Steps: ", args.n)
    print("use_pred: ", args.pred)
    # print("max_error_bound: ", args.err)

    main(args.p, args.g, args.n, args.pred)#, args.err)