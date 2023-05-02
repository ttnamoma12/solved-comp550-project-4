Download Link: https://assignmentchef.com/product/solved-comp550-project-4
<br>
<strong>Algorithmic Robotics</strong>

<h1>Out of Control Planning</h1>

<strong>The documentation for </strong><strong>OMPL </strong><strong>can be found at </strong><a href="http://ompl.kavrakilab.org/"><strong>http://ompl.kavrakilab.org.</strong></a>

In the previous project, you computed motion plans for a <em>rigid body </em>that was able to move in any direction and at any speed, so long as the path was collision free. In this project, you will be planning for more complex systems – systems with dynamical constraints on their motion. For these complex systems, a collision-free path may not always be feasible. As an example, consider a car trying to parallel park. Although the a <em>straight-line </em>path sideways to the parking spot may be collision free, the car cannot translate horizontally and must execute a longer path to reach the parking spot. The fact that the car cannot directly move sideways implies that the car is a <em>non-holonomic </em>system; the system cannot control its position directly, and has constraints that include terms other than its position. This is a key point that differentiates planning between geometric and dynamic problems.

In this project you will plan motions for non-holonomic systems whose dynamics are described by an ordinary differential equation of the form <em>q</em>˙ = <em>f</em>(<em>q</em><em>,u</em>)<em>, </em>where <em>q </em>is a vector describing the current state of the system and <em>u </em>is a vector of control inputs to the system. The systems you will plan for are a torque-controlled pendulum and a car-like vehicle moving in a street environment. Dynamically feasible and collision-free motions for these systems will be computed using the RRT and KPIECE planners that are already implemented in OMPL. Additionally, you will also learn about and implement a new planner called Reachability-Guided RRT (RG-RRT), one of many variants of RRT.

<h2>A Torque-controlled Pendulum System</h2>

A pendulum is one of the simplest dynamic systems. The state <em>q </em>= (<em>θ</em><em>,ω</em>) of the system is described by the orientation of the pendulum (<em>θ</em>) and its rotational velocity (<em>ω</em>). Assume that <em>θ </em>= 0 corresponds to the bar of the pendulum being horizontal. The objective is to move the pendulum from an initial state of hanging down at rest to a final state of pointing up at rest:

→

A motor can apply a torque <em>τ </em>to the axis of rotation of the pendulum. The pendulum’s dynamics are described by the following differential equation (<em>g </em>is gravity, ≈ 9<em>.</em>81):

˙  Figure 1: A possible solution trajectory for the <em>q</em>˙pendulum in phase space. <em>From Shkolnik et al.;</em>

<em>see below.</em>

When an arbitrarily large torque can be applied, the planning problem is not difficult as the torque can hold the system <em>quasi-statically </em>at any configuration. Many real manipulators use this strategy and utilize very large torques, allowing them to plan geometrically. However, is this always the case? In reality, the pendulum’s motor can source only a finite torque.

In your implementation, use 3, 5, and 10 as the absolute value of the upper and lower bounds on the torque limits (i.e., <em>τ </em>∈ [−3<em>,</em>3] for a torque bound of 3). Use 10 as absolute value of the rotational velocity limit. Figure 1 shows a possible solution on the (<em>θ</em><em>,ω</em>) phase space of the pendulum. The start pose is at the center of the spiral and either one of the red circles are valid end poses.

<h2>A Car-like system</h2>

The second system involves a simple vehicle moving in a street environment, an example of which is shown in Figure 2. The state of the vehicle is represented by its position (<em>x</em><em>,y</em>), heading <em>θ</em>, and forward velocity <em>v</em>:

<em>q </em>= (<em>x</em><em>,y</em><em>,θ</em><em>,v</em>)

Note that forward velocity <em>can </em>be negative, in which case the car moves in reverse. The control inputs to this system <em>u </em>consist of the angular velocity <em>ω </em>and the forward acceleration of the vehicle ˙<em>v</em>,

<em>u </em>= (<em>ω</em><em>,v</em>˙)

Both terms of <em>u </em>are bounded in magnitude. The system dynamics are described by:

<em>x</em>˙ <em>v</em>cos<em>θ</em>

<em>q</em>˙ = <sub></sub><em>y</em>˙<sub> </sub>= <sub></sub><em>v</em>sin<em>θ</em><sub></sub>

<table width="461">

 <tbody>

  <tr>

   <td width="269"></td>

   <td width="192">Figure 2: Vehicle navigating in street environment. <em>From Shkolnik et al.; see below.</em></td>

  </tr>

 </tbody>

</table>

Similar to the pendulum system, the car cannot source an arbitrarily large acceleration or angular velocity. You need to set bounds on the control space to ensure a dynamically feasible trajectory.

<h2>Planning for Dynamical Systems</h2>

The key difference when planning for dynamical systems is that the <em>input space </em>of controls must be sampled

(a <em>control space</em>), along with a duration to apply the control. In the rigid body case, this input space was simply the configuration space since the motion between any two configurations is feasible. For dynamical systems, a control input is applied for a prescribed time, and the equations of motion are applied to compute the resulting configuration. The planners in OMPL for dynamic systems sample controls that are applied for some small period of time. It is thus necessary to integrate the equation <em>q</em>˙ = <em>f</em>(<em>q</em><em>,u</em>) to get the resulting state. OMPL includes support for numerical integration and you “only” have to implement <em>f</em>(<em>q</em><em>,u</em>). A detailed example is given at <a href="http://ompl.kavrakilab.org/odeint.html">http://ompl.kavrakilab.org/odeint.html</a><a href="http://ompl.kavrakilab.org/odeint.html">.</a>

You must construct the correct state space for each of the systems, as well as a correct StateValidityChecker. For the pendulum system, you can assume an environment without obstacles. However, the state validity checker must ensure that the angular velocity of the pendulum is within the bounds that you specify. Similarly you must verify that the velocity of the vehicle is within the bounds you set. For simplicity, you can assume that the vehicle is a point system for collision checking purposes, but collision checking methods from the previous project can also be used to add geometry to the vehicle if you wish.

See the OMPL demo programs for some examples of how to put all the different pieces together. Look in particular at ${OMPL DIR}/share/ompl/demos/RigidBodyPlanningWithODESolverAndControls.cpp (also available online on the “demos” page).

<h2>Reachability-guided RRT</h2>

An article by Shkolnik et al. (2009) proposes a motion planning algorithm for dynamic systems. The authors propose that an RRT should only be grown towards a random state that is likely to be <em>reachable </em>from its nearest neighbor in the tree. For dynamic systems, states that are near a given state are not always easily reachable. For instance, vehicle systems cannot easily move sideways. You will use the information in their article (summarized below) to understand the problem and the performance characteristics of the systems they assessed. Next, you will implement the motion planner they describe, the <em>reachability-guided </em>RRT (RG-RRT) and test its performance.

The main change of RG-RRT over RRT is that for each state <em>q </em>in the tree, an approximation of the <em>reachable set </em>of states, <em>R</em>(<em>q</em>), is maintained. The reachable set <em>R</em>(<em>q</em>) is the set of states the system can arrive at, starting at state <em>q</em>, after applying any valid control(s) for a small period of time (a fixed value you set). You can approximate <em>R</em>(<em>q</em>) by choosing a small number of controls and computing the states that were reached after applying them for a short duration. For the pendulum you should pick 11 evenly spaced values for <em>τ </em>between the torque limits (i.e., {-10, -8, …, 8, 10}). For the car you can do the same for <em>u</em><sub>0 </sub>(you can ignore <em>u</em><sub>1 </sub>for the reachable set). Next, apply each of the controls to the start state <em>q </em>for a small time step to obtain <em>R</em>(<em>q</em>) (use the SpaceInformation::propagate method for this). You can store the reachable set approximation by extending Motion objects (such as those in RRT) to also store the reachable set (as a vector of states). You can assume for this project that the control space is a RealVectorControlSpace with bounds. Please make sure you write your assumptions in your report.

The next step is to modify nearest neighbor queries. Given a random state <em>q</em><sub>rand</sub>, you need to find the state <em>q</em><sub>near </sub>that is closest to <em>q</em><sub>rand </sub>and has a state in <em>R</em>(<em>q</em><sub>near</sub>) that is closer to <em>q</em><sub>rand </sub>than <em>q</em><sub>near</sub>. An easy strategy is to use the NearestNeighborLinear data structure that contains Motion objects (such as those in RRT) with a special distance function (as described above) that you need to write. This distance function returns the distance between two states <em>q</em><sub>0 </sub>and <em>q</em><sub>1 </sub>when <em>q</em><sub>1 </sub>is reachable from <em>q</em><sub>0 </sub>(i.e., there exists a state in <em>q<sup>r </sup></em>∈ <em>R</em>(<em>q</em><sub>0</sub>) that is closer to <em>q</em><sub>1 </sub>than <em>q</em><sub>0 </sub>is to <em>q</em><sub>1</sub>) and returns ∞ otherwise. Note that the reachability set is <em>only </em>used for nearest neighbor queries.

To implement RG-RRT, we recommended you start with the RRT implementation from the OMPL.app source distribution. These files are found in omplapp/ompl/src/ompl/control/planners/rrt if you compiled from source (or the virtual machine). The files are also found at <a href="https://bitbucket.org/ompl/ompl">https://bitbucket.org/ompl/ompl</a><a href="https://bitbucket.org/ompl/ompl">.</a>

<h2>Projections</h2>

KPIECE (among other planners) requires the definition of a projection for the state space being planned in. KPIECE uses the projection to estimate how well different parts of the state space have been sampled. For the existing state spaces in OMPL a projection is already defined; however, a good projection often requires some information about the specific robot and task. Although projections are generally a reduction in dimension, projections in OMPL do not have a limit on how many dimensions you wish to define nor do they require that you chose a subset of the dimensions in your state space.

For example, one of the dimensions in your projection could even be the Cartesian position of some point on the robot. Often times, a desirable projection might be one which has a strong correlation with progress toward the goal or indication of greater coverage in the workspace. With control spaces, such a projection usually includes some information from the dynamic dimensions in the state.

<h2>Project exercises</h2>

<ol>

 <li>Implement the state validity checker and differential equations for the pendulum and the vehicle systems described above. Solve the motion planning problems described for these systems using the RRT Visualize the solution paths to make sure they are correct and to compare the solution paths found using torque values of 3, 5, and 10 for the pendulum problem.</li>

 <li>Extend the program from #1 to solve the pendulum and the vehicle problems using the KPIECE You will need to define a projection for the state spaces you create for the pendulum and the car. See <a href="http://ompl.kavrakilab.org/projections.html">http://ompl.kavrakilab.org/projections.html</a> for details on how to define a projection and associate it</li>

</ol>

with a state space.

<ol start="3">

 <li>Implement RG-RRT (see Scholnik et al., 2009) and solve the pendulum and vehicle problems as in #1 and #2. Make sure to visualize the solution paths.</li>

 <li>Compare your RG-RRT against RRT and KPIECE using the OMPL Benchmark class for both the car and pendulum environments. Use a torque value of 3 for the pendulum. Any conclusions must come from at least 20 independent runs of each planner.</li>

</ol>

<h2>Protips</h2>

<ul>

 <li>Be sure to use the ompl::control namespace instead of ompl::geometric in this project. This includes the SimpleSetup class and all of the planners.</li>

 <li>Solution paths from the <em>control-based </em>planners in OMPL contain more information than their geometric counterpart. You can “geometrize” a controlled path using the asGeometric method. This will return a PathGeometric object that you can use with your existing visualization tools.</li>

 <li>Make sure to perform state validity checking for the reachable set in your RG-RRT If a state is not valid, it certainly is not <em>reachable</em>.</li>

 <li>It is <em>highly </em>unlikely that a control-based planner will be able to find a goal configuration exactly.</li>

</ul>

Think about why this is the case. The call to SimpleSetup.setStartAndGoalStates() has a third parameter that defines an acceptable radius around the goal state. Any state that is within this radius

will be considered an exact goal state. You will need to play with this parameter to successfully plan.

<ul>

 <li>Generally, geometric planners (such as RRT and your Random Tree Planner from the last assigment) use NearestNeighborsGNAT, which requires a distance function that is a <em>metric</em>. However, for RG-RRT, the distance defined by the reachability set (∞ if it cannot be reached) is assymetric which breaks the necessary metric properties to use NearestNeighborsGNAT. For this reason, you must use</li>

</ul>

NearestNeighborLinear to build a nearest-neighbor structure for RG-RRT.

<ul>

 <li>In this exercise you will have to generate new states for the reachability set of a state. To do this, you want to apply some control to an existing state. You can allocate controls by allocating new controls from ompl::control::SpaceInformation using allocControl. Just like with states, you can cast controls to their control type defined in the control space. For example, RealVectorControlSpace has RealVectorControlSpace::ControlType. Use ompl::control::SpaceInformation to generate new states through propogate, which applies a control to get a new state.</li>

</ul>

<h2>Deliverables</h2>




You are also expected to complete a progress report due due Thursday October 31st at 1pm. This progress report should be short, no longer than one page in PDF format. At a minimum, the report should state who your partner is and what progress you have made thus far.

To submit your project, clean your build space with make clean, zip up the project directory into a file named Project3 &lt;your NetID&gt; &lt;partner’s NetID&gt;.zip, and submit to Canvas. Your code must compile within a modern Linux environment. If your code compiled on the virtual machine, then it will be fine. If you deem it necessary, also include a README with details on compiling and executing your code. In addition to the archive, submit a short report that summarizes your experience from this project. The report should be anywhere from 1 to 5 pages in length in PDF format, and contain the following information:

<ul>

 <li>(4 points) A succinct statement of the problem that you solved.</li>

 <li>(2 points) Images of your environments and a description of the start-goal queries you are evaluating.</li>

 <li>(2 points) A short description of the robots (their geometry) and configuration spaces.</li>

 <li>(10 points) Explain the differences in solution paths when torque is limited to 3, 5, and 10 for the pendulum problem.</li>

 <li>(10 points) A summary of benchmarking data for the RRT, KPIECE and RG-RRT planners for both systems. Use a torque value of 3 for the pendulum for the other benchmark and comparison exercises. This data must be presented in a quantitative manner (i.e., plots or tables) and any conclusions must be supported from this data. Consider the following metrics: computation time, path length, number of tree nodes, and success rate. When referencing benchmarking results you must include the referenced data as figures.</li>

 <li>(10 points) A head-to-head comparison of RRT and RG-RRT. Discuss the performance trade-offs of RG-RRT for computing reachable states in terms of the <em>length </em>of the time period and the <em>number </em>of controls. Support your claims with images and/or benchmark data.</li>

 <li>(2 points) Rate the difficulty of each exercise on a scale of 1–10 (1 being trivial, 10 being impossible). Give an estimate of how many hours you spent on each exercise, and detail what was the hardest part of the assignment. Additionally, for students who completed the project in pairs, describe your individual contribution to the project.</li>

</ul>

Additionally, you will be graded upon your implementation. Your code must compile, run, and solve the problem correctly. Correctness of the implementation is paramount, but succinct, well-organized,

well-written, and well-documented code is also taken into consideration. Visualization is an important component of providing evidence that your code is functioning properly. The breakdown of the grading of the implementation is as follows:

<ul>

 <li>(35 points) RG-RRT</li>

 <li>(10 points) Pendulum Problem</li>

 <li>(10 points) Car Problem</li>

 <li>(5 points) Benchmark</li>

</ul>

Those completing the project in pairs need only provide one submission.

<h2>Reference</h2>

<ol start="2009">

 <li>Shkolnik, M. Walter, and R. Tedrake, Reachability-guided sampling for planning under differential constraints, in <em>IEEE Intl. Conf. on Robotics and Automation</em>, pp. 2859–2865, 2009. <a href="https://dx.doi.org/10.1109/ROBOT.2009.5152874">http://dx.doi.org/10.</a></li>

</ol>

<a href="https://dx.doi.org/10.1109/ROBOT.2009.5152874">1109/ROBOT.2009.5152874</a><a href="https://dx.doi.org/10.1109/ROBOT.2009.5152874">,</a> <a href="https://dspace.mit.edu/openaccess-disseminate/1721.1/61653">http://dspace.mit.edu/openaccess-disseminate/1721.1/61653</a>