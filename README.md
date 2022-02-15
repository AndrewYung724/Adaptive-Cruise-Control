# Adaptive-Cruise-Control
Design of a game theoretic adaptive cruise control algorithm using a one player dynamic game model with full information structure. We apply a receding horizon control algorithm for predicting the actions of other players/drivers to yield the optimal acceleration.

## Model Assumptions

For this project we contrain the autonomous driving challenge to one-dimension where the autonomous driver is simply attempting to achieve its set speed while avoiding collisions.

<p align="center">
<img width="400" alt="model" src="https://user-images.githubusercontent.com/38053500/154119192-a5beb4db-27c0-49e9-9492-d5ec3c4cff09.PNG">
<p align = "center">
Fig.1 - Simplified Adaptive cruise control model.
</p>

With full information structure the autonomous driver (in red) is able to read the following state variables:

![image](https://user-images.githubusercontent.com/38053500/154146269-56166ff4-fda1-4b33-be16-6107cf9921d6.png)
<p align = "center">
Fig.2 - Model state variables
</p>




### Dependencies

* Describe any prerequisites, libraries, OS version, etc., needed before installing program.
* ex. Windows 10

### Installing

* How/where to download your program
* Any modifications needed to be made to files/folders

### Executing program

* How to run the program
* Step-by-step bullets
```
code blocks for commands
```

## Help

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
