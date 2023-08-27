# CBF-RRT
## Publication
```
@inproceedings{10.1145/3365265.3365282,
author = {Yang, Guang and Vang, Bee and Serlin, Zachary and Belta, Calin and Tron, Roberto},
title = {Sampling-Based Motion Planning via Control Barrier Functions},
year = {2019},
isbn = {9781450372886},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3365265.3365282},
doi = {10.1145/3365265.3365282},
abstract = {Robot motion planning is central to real-world autonomous applications, such as self-driving cars, persistence surveillance, and robotic arm manipulation. One challenge in motion planning is generating control signals for nonlinear systems that result in obstacle free paths through dynamic environments. In this paper, we propose Control Barrier Function guided Rapidly-exploring Random Trees (CBF-RRT), a sampling-based motion planning algorithm for continuoustime nonlinear systems in dynamic environments. The algorithm focuses on two objectives: efficiently generating feasible controls that steer the system toward a goal region, and handling environments with dynamical obstacles in continuous time. We formulate the control synthesis problem as a Quadratic Program (QP) that enforces Control Barrier Function (CBF) constraints to achieve obstacle avoidance. Additionally, CBF-RRT does not require nearest neighbor or explicit collision checks during sampling.},
booktitle = {Proceedings of the 2019 3rd International Conference on Automation, Control and Robots},
pages = {22–29},
numpages = {8},
keywords = {Sampling, Motion Planning, Control},
location = {Prague, Czech Republic},
series = {ICACR 2019}
}
```
[Arxiv Paper](https://arxiv.org/abs/1907.06722)

[ACM Paper](https://dl.acm.org/doi/10.1145/3365265.3365282)

## Dependenicies
* Python 3.7.3
* Numpy 1.16.2
* Gurobi 8.1.1 (Free for education usages: https://www.gurobi.com/)


## Files
*cbf_rrt_linsys.py* run this file driectly to see an example of CBF-RRT on a point mass. 
*simulation_closedform_linsys.py* performs manual intergration over the point mass dynamics. 

