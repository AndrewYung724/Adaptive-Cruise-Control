# Adaptive-Cruise-Control
Design of a game theoretic adaptive cruise control algorithm using a one player dynamic game model with full information structure. We apply a receding horizon control algorithm for predicting the actions of other players/drivers to yield the optimal acceleration.

## Model Assumptions

For this project we constrain the autonomous driving challenge to one-dimension where the autonomous driver is attempting to achieve its set speed while avoiding collisions.

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
<img width="750" alt="model" src="https://user-images.githubusercontent.com/38053500/154816420-51d9446e-1e78-4ba2-8078-8a1bdb78b117.png">
<p align = "center">
Fig.3 - One-Player Dynamic Game formulation
</p>
Where the action u<sub>k</sub> represents the action taken at time step k. However, how is the optimal u<sub>k</sub> that best acheives the objectives of avoiding collisions and reaching its velocity setpoint? For this challenge, we introduce a performance metric known as the cost function J(x<sub>k</sub>,u<sub>k</sub>) 


### Cost Function

The cost function should penalize two main phenomena:
1. Events where a collision is likely
2. Deviations from our velocity setpoint

#### Penalizing Likely Collisions
For measuring the likelihood of a potential crash we use the time-to-collision metric (TTC) whose derivation can be shown in figure 4.

<p align= "center">
<img width="700" alt="model" src="https://user-images.githubusercontent.com/38053500/154816888-e5a50295-161b-471e-8ab1-b0787c175366.png">
<p align = "center">
Fig.4 - TTC Derivation
</p>

This derivation simply takes care of the positive and negative cases of relative distance and velocity so we reach a compact form of the metric.

With our TTC defined, we insert the metric into an asymptotic function to map the TTC to a cost scalar.

<p align= "center">
<img width="400" alt="model" src="https://user-images.githubusercontent.com/38053500/154817327-7b958a20-897b-4d34-acf7-0beff371bb75.png">
<p align = "center">
Fig.5 - TTC to Cost Mapping
</p>

This asymptotic function enables us to exponentially penalize the TTC as it approaches zero (i.e. a collision). α in this equation is a weight that controls when the TTC threshold should start exploding to large scalars.

When we subsitute the TTC for the front and rear cars into this function we obtain the following cost function for avoiding collisions.

<p align= "center">
<img width="400" alt="model" src="https://user-images.githubusercontent.com/38053500/154817742-21d03e5f-ff40-4659-9e27-91be29ab8ced.png">
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
<img width="400" alt="https://user-images.githubusercontent.com/38053500/154821427-d5083a22-7505-4901-b872-3416b4bfc0c6.png">
<p align = "center">
Fig.8 - Complete ACC Cost Function.
</p>

### Finding the Optimal Action
Intuitively, if the action commits to the maximum acceleration it crashes into the front vehicle and if it commits to the minimum acceleration the rear vehicle will crash into it. In both cases the cost function explodes to infinity. So how do we find the middle ground that minimizes the cost function? We implement a receding horizon control algorithm that works as follows:
1. Discretize the action space u∈[-a,a] into M accerlations
2. Read the state variables x<sub>k</sub>
3. At time step k, simulate N time steps ahead for all possible actions, assuming the front and rear vehicles will commit to the same velocity for those time steps.
4. Evaluate the states for each action time step pair.
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
<img width="400" alt="model" src="https://user-images.githubusercontent.com/38053500/154822135-ca373dd4-1e67-47a5-bcc4-4ad89e4de344.png">
<p align = "center">
</p>

5. Evaluate the cost function for all states
<p align= "center">
<img width="400" alt="model" src="https://user-images.githubusercontent.com/38053500/154822179-04c3858d-d2dc-4f5e-8cbe-9266529321e9.png">
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
6. Sum across the time steps and select the jth action that yields the lowest cost
\\
<p align= "center">
<img width="700" alt="model" src="https://user-images.githubusercontent.com/38053500/154822499-23b8c325-e618-4b4b-93bb-8ebcadad7298.png">
<p align = "center">
</p>

7. Apply action and iterate for next step.

### Results

For simulation we assumed the surrounding "human" drivers to operate under a 30m/s sinusodal velocity 

### Executing program

* How to run the program
* Step-by-step bullets
```
code blocks for commands
```

## Future Improvements

Any advise for common problems or issues.
```
command to run if program contains helper info
```

## Authors

Contributors names and contact info

ex. Dominique Pizzie  
ex. [@DomPizzie](https://twitter.com/dompizzie)

## Version History

* 0.2
    * Various bug fixes and optimizations
    * See [commit change]() or See [release history]()
* 0.1
    * Initial Release

## License

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details

## Acknowledgments

Inspiration, code snippets, etc.
* [awesome-readme](https://github.com/matiassingers/awesome-readme)
* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2)
* [dbader](https://github.com/dbader/readme-template)
* [zenorocha](https://gist.github.com/zenorocha/4526327)
* [fvcproductions](https://gist.github.com/fvcproductions/1bfc2d4aecb01a834b46)
