# Adaptive-Cruise-Control
Design of a game theoretic adaptive cruise control algorithm using a one player dynamic game model with full information structure. We apply a receding horizon control algorithm for predicting the actions of other players/drivers to yield the optimal acceleration.

## Model Assumptions
For this project we constrain the autonomous driving challenge to one-dimension where the autonomous driver's objective is to achieve its set speed while avoiding collisions.

<p align="center">
<img width="400" alt="model" src="https://user-images.githubusercontent.com/38053500/154119192-a5beb4db-27c0-49e9-9492-d5ec3c4cff09.PNG">
<p align = "center">
Fig.1 - Simplified Adaptive cruise control model.
</p>

With full information structure the autonomous driver (in red) is able to read the following state variables:

<p align= "center">
<img width="400" alt="model" src="https://user-images.githubusercontent.com/38053500/154146269-56166ff4-fda1-4b33-be16-6107cf9921d6.png">
<p align = "center">
Fig.2 - Model state variables
</p>

With the state variables defined we can now formulate our ACC one-player dynamic game.
<p align= "center">
<img width="750" alt="model" src="https://user-images.githubusercontent.com/38053500/155250164-bff7c565-7248-42df-9f8f-c22b1fab3d81.png">
<p align = "center">
Fig.3 - One-Player Dynamic Game formulation
</p>
Where the action u<sub>k</sub> represents the action taken at time step k. However, how is the optimal u<sub>k</sub> that balances our objectives determined? For this challenge, we introduce a performance metric known as the cost function J(x<sub>k</sub>,u<sub>k</sub>) 


### Cost Function
The cost function should penalize two main phenomena in this scenario:
1. Events where a collision is likely
2. Deviations from our velocity setpoint

#### Penalizing Likely Collisions
For measuring the likelihood of a potential crash we use the time-to-collision metric (TTC) whose derivation can be shown in figure 4.

<p align= "center">
<img width="700" alt="model" src="https://user-images.githubusercontent.com/38053500/154816888-e5a50295-161b-471e-8ab1-b0787c175366.png">
<p align = "center">
Fig.4 - TTC Derivation
</p>

TTC is simply the relative velocity over relative distance. This derivation only taking care of the positive and negative cases of relative distance and velocity so we reach a compact and correct form of the metric. The TTC metric is robust against scenarios where the driver has a vehicle converging on his/her tail at a high speed. It also works in the best interest of traffic overall since it deters against any premature breaking.

With our TTC defined, we insert the metric into an asymptotic function to map the TTC to a cost scalar.

<p align= "center">
<img width="400" alt="model" src="https://user-images.githubusercontent.com/38053500/154817327-7b958a20-897b-4d34-acf7-0beff371bb75.png">
<p align = "center">
Fig.5 - TTC to Cost Mapping
</p>

This asymptotic function enables us to exponentially penalize the TTC as it approaches zero (i.e. a collision). α in this equation is a weight that controls when the TTC threshold should start exploding to large scalars (i.e. start penalizing when TTC<15 seconds).

When we subsitute the TTC for the front and rear cars into this function we obtain the following cost function for avoiding collisions.

<p align= "center">
<img width="550" alt="model" src="https://user-images.githubusercontent.com/38053500/154817742-21d03e5f-ff40-4659-9e27-91be29ab8ced.png">
<p align = "center">
Fig.6 - Collsion  likelihood cost function
</p>

#### Penalizing Deviation from velocity
The cost of deviation from our velocity can be represented simply through the following equation:

<p align= "center">
<img width="250" alt="model" src="https://user-images.githubusercontent.com/38053500/154817842-187c561d-c5e7-4d04-bdc8-9d6bd9c7c1b6.png">
<p align = "center">
Fig.7 - Velocity Setpoint Cost Function
</p>

Where β represents a weight on how much we want the car to prioritize reaching the setpoint.

#### Cost Function Construction
Finally, we acheive our cost function for which we want to minimize subject to our action u<sub>k</sub>. 

<p align= "center">
<img width="700" alt="model" src="https://user-images.githubusercontent.com/38053500/154821427-d5083a22-7505-4901-b872-3416b4bfc0c6.png">
<p align = "center">
Fig.8 - Complete ACC Cost Function.
</p>

## Finding the Optimal Action: Receding Horizon Control
Intuitively, if the action commits to the maximum acceleration it crashes into the front vehicle and if it commits to the minimum acceleration the rear vehicle will crash into it. In both cases the cost function explodes to infinity. So how do we find the middle ground that minimizes the cost function? We implement a receding horizon control algorithm that works as follows:
1. Discretize the action space u∈[-a<sub>max</sub>,a<sub>max</sub>] into M distinct accerlations
2. Read the state variables x<sub>k</sub>
3. At time step k, simulate N time steps ahead for all possible actions, assuming the front and rear vehicles will commit to the same velocity for those time steps. Evaluate the states for each action time step pair.
<!--
N \text{ Time Steps}
\begin{cases}
\overbrace{
\begin{pmatrix}
 x_{11} & \cdots & x_{1M}\\
 \vdots & \ddots & \vdots\\
 x_{N1} & \cdots & x_{NM}
 \end{pmatrix} }^{\bf{\underline{U}}}
\end{cases}
-->

<p align= "center">
<img width="400" alt="model" src="https://user-images.githubusercontent.com/38053500/155251147-25af8547-7731-4229-bc8d-c36f70f909ab.png">
<p align = "center">
</p>

4. Evaluate the cost function for all states
<p align= "center">
<img width="500" alt="model" src="https://user-images.githubusercontent.com/38053500/154822179-04c3858d-d2dc-4f5e-8cbe-9266529321e9.png">
<p align = "center">
</p>
<!--
\begin{pmatrix}
 J( \underline{x} _{11}) & \cdots & J(\underline{x}_{1M})\\
 \vdots & J( \underline{x} _{ij}) & \vdots\\
 J(\underline{x}_{N1}) & \cdots & J(\underline{x}_{NM})
 \end{pmatrix}
 \Longrightarrow 
 \begin{pmatrix}
 \sum_{i=1}^N J( \underline{x}_{i1})& \cdots & \sum_{i=1}^N J( \underline{x}_{iM})\\
 \end{pmatrix} 
-->
5. Sum across the time steps and select the jth action that yields the lowest cost

<p align= "center">
<img width="700" alt="model" src="https://user-images.githubusercontent.com/38053500/154822499-23b8c325-e618-4b4b-93bb-8ebcadad7298.png">
<p align = "center">
</p>

6. Apply action and iterate for next step.

## Results

For simulation we assume the following state initial condition and model parameters.
<p align="center"></p>
<table>
<tr><th> Initial States </th><th>Model Parameters</th></tr>
<tr><td>

|State| Value|
|--|--|
 |v<sub>l</sub>| 30 m/s |
 |v<sub>r</sub>| 30 m/s |
 |v<sub>e</sub>| 30 m/s |
 |x<sub>l</sub>| 60 m |
 |x<sub>r<rsub>| 6 m |
</td><td>

|Parameter|Value| 
|--|--|
|α|0.0005|
|β|5000|
|V<sub>ref</sub>| 100 m/s|
|N| 2 s| 
 |a<sub>max</sub> | 3.5 m/s<sup>2</sup>
 |a<sub>l</sub> = a<sub>r</sub> | sin(0.3t) (67-81mph) |

</td></tr> </table>
<p></p>


https://user-images.githubusercontent.com/38053500/155233557-6616bdc3-0f01-4ce4-a663-8e377ac81273.mp4

<p align = "center">
Fig.9 - Adaptive Cruise Control Demonstration
</p>


From this result we conclude that the ACC is a success! The ego vehicle working under the ACC algorithm, appropriately applies its maximum acceleration to avoid being rear ended. After avoiding a collision we can see that it is attempting to reach its setpoint of 100m/s but is limited to the front vehicle's position.
 
## Summary
 In summary, this ACC algorithm is simulates the performance of all actions before selecting the best action. During this simulation the algorithm assumes the other agents will commit to their speeds for the next N time steps and evaluates the states for all possible actions for N time steps. Once the states are calculated through simple kinematic equations, we evaluate the performance of action by applying the cost function along the trajectory of each action. Then we select the action associated with best performance (i.e. lowest cost). 

 
## Future Improvements

With the logic of the game theoretic adaptive cruise control in place, some future improvements for this ACC include:
 - Enable ego car to detect front and rear relative distances instead of just passing the values through in simulation
 - Make distance and velocity estimates robust to noise/disturbances
 - Propperly car's gas -> position plant
 - Remodel cars to occupy some distances instead of just point agents
 - Can include objective of conserving fuel or minimize braking
 - Add additional minimum safety distances for margin of error
 - Accurate models of humans and their driving policy
 
## Acknowledgments
This was the final project of UCSB's ECE 270 course, Non-Cooperative Game Theory, taught by Professor Joao Hespanha fall 2021.
 
