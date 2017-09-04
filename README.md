# Build a Model Predictive Controller
## CarND-Controls-MPC
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
---

## Overview
  The purpose of this project is to build a Model Predictive Controller to drive a virtual car around the Udacity CarND simulator's lake track, using the content built during the MPC quizes and topics covered previously such a coordinate rotation.  The project began with a code framework provided by the Udacity team, with specific sections left as TODO's for the student to complete.  The bulk of the code was taken from the MPC quiz code, which had been developed in the lesson just prevously.  In the project, this code was tied to the simulator used througout this term, cost function weights modified, and actuator latency accounted for.  The output of the project code are JSON messages which allow the simluator to drive the virtual car around the lake track, display the road's target trajectory(yellow), along with the MPC's planned trajectory(green).  A video recording of a successful run around the track can be seen here: https://youtu.be/6fz-J8Ld4Wg 

  The build instructions follow, with the Rubric at the end of this README.  

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x, which can work with modifications as described on this [Udacity support forums post](https://discussions.udacity.com/t/windows-linux-build-using-ipopt-3-11-x-installation/341644?source_topic_id=355940) otherwise, download 3.12.x and run  `sudo bash install_ipopt.sh C:/downloaddir/Ipopt-3.12.x` to install Ipopt. [Ipopt releases page](https://www.coin-or.org/download/source/Ipopt/).
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Rubric

The Rubric is a set of goals the project code must meet.  I address each one here.  
1) The code must compile with "cmake" and "make.   
   The code does compile with cmake and make, and all warnings encountered have been cleaned up as well.

2) The Model.  (Describe the model in detail)  
  The model contains 6 features stored in a vector, along with 2 actuators.  The state includes the x and y position, the angle psi, the cross track error, and the psi error[x,y,ψ,v,cte,eψ].  The two actuator values are the delta(steering) and accelleration(throttle)[δ,a].

  The model attempts to predict a desired future state for the vehicle N time steps into the future, relying on a target trajectory and a cost function.  The model attempts to minimize the result of this cost function, which includes a number of factors about the car's behavior.  The most important of these factors is the error between the car's position and the desired lane centerline, called the "cross track error" or CTE.  If the CTE is too great, the car ends up off the road and in a ditch.  It is therefor critical to minimize the CTE.

  The CTE is not the only important part of the cost function, however.  The angle/heading of the vehicle, the speed, and the degree of change in steering are all critical to ensuring that the car stays on the road, and the passengers are not injured during the trip.  In trying to reach the centerline and minimize the CTE, too great of a steering angle will result in swerving, and too great of a change in throttle will result in seat-belt bruises.  For these reasons, the CTE and psi error are weighted in the cost function as the largest influencers, with the steering and throttle also included, but with smaller weights.  This slows the rate of change of any aspect of the vehicle's behavior, while aiming for a centered and stable trip.  To avoid the chance that this cost function finds a local minimum and simply stops in the middle of the road, we also penalize the vehicle for traveling at any speed below a target reference velocity, here set to 60.

  Behind the scenes of the model are a number of vectors containing all the state features and constraints used to calculate the desired path.  The vars vector includes N * the number of state features plus the 2 actuators we are using, holding the projected future states predicted by the model.  The fg vector contains the cost function at fg[0], plus the vars content and the constraints we will define on the model.

  Ipopt expects equations with an upper and lower bound for the acceptable solutions.  For most of our purposes, we are attempting to find 0 difference, denoting an equality.  So instead of calculating x1, we find the difference between the projected x1 and the calculation used to find it:  x1 - (x0 + v0 * CppAD::cos(psi0) * dt) .

  We then use the provided polyfit equation to fit a third order polynomial to the waypoints, and get back a vectors of coefficients, which are in turn used to find the cross track error and psi error.  This process is then repeated, using the letancy-controlled state to re-initialize the predicted trajectory, and re-determine the CTE and psi error once again.

  Additionally, some portions of the model will be limited in reality to certain possible values.  For this reason, we limit the possible steering angle to [-25, 25] degrees, as in reality, a 100 degree steering angle would likely break the internal mechanisms of the car's steering.

3) Timestep Length and Elapsed Duration (N & dt)  
  In my initial setup, N was set to 20, with a dt of 0.05.  This lead to a fairly chaotic result, with the car eventually running off the track.  I set about shortening the N value per the recomendation in the lessons of "as large as possible [but not too large]".  Moving to N=15 and dt of 0.06667, the results were improved but still not good enough.  I tried N=10 and dt=0.01 as suggested in the Q&A video, and that value pair worked.  I also tried values lower than 10 for N, with decresing effectiveness as the number got smaller - for example, an N of 6 resulted in a successful run of the track, but the car hung to the right shoulder the entire time. 

4) Polynomial Fitting and MPC Preprocessing  
   The polyfit() function provided in the framework code was used to fit a third order polynomial to the waypints of the target trajectory.  The only major preprocessing done was to rotate from the global to car coordinate space.

5) Model Predictive Control with Latency  
  The project induded a requirement to handle artificial latency added into the code by a call to the sleep() function, pausing the system for 100 milliseconds.  This delay was added to model the real-world delays that occur between issuing a command to a actuator and the actuator completing that command, including all latency in communication channels in between.  To account for this delay, I have altered the model's state to include a 0.1s offset, in place of the 0's mentioned in the Q&A video.  This state was fed to the solver instead of the current state, effectively making decisions based on the car's future state rather than the current state.

  This also required using the full formula for calculating the error psi, in place of the truncated version discussed in the Q&A video.  Per the video, the truncated version was possible due to the px, py and psi values being 0; handling the offset meant this was no longer true.

6) The vehicle must successfully drive a lap around the track.  
  With the current cost function weights, the car completes a lap of the track.  See https://youtu.be/6fz-J8Ld4Wg for a 2-lap recording of the MPC in practice.

## Conclusion

  The MPC project was a difficult one, mostly due to the translation between theory and implementation through the Ipopt and CppAD libraries.  These had not been introduced prior to the MPC quizes, and installing them proved difficult; it took about 6 hours to get them mostly working in the configuration I have been using for Term 2, partially because the coin-or servers were down.  Due to my use of Docker, the quiz code's expectation of being able to display matplotlib graphs never worked; I will need to figure out how to pass the X11 socket to the host machine to display graphical output.  It seems that the quiz author expected people would be using linux exclusively, or the linux userspace on Widnows 10.

  The other difficult aspect was applying the lesson's information on constraints to how Ipopt actually works.  While I understood the lesson's specific content, how to translate that into functional code was not clear to me at all by the time the MPC quiz apeared.  My confusion was helped greatly by the Q&A video and the student forum discussions, of which there were many.  This suggests a general confusion on this topic.  The simulator accepting turning values between -1 and 1, but returning angles in radians, and then needing to invert the signs on that out in order to use them added significant confusion to the process.  The lecture here helped somewhat:  https://www.youtube.com/watch?v=1nfl6OM32-4&t=523s , but there is still a fair amount of the MPC process which seems unclear to me at this point.  Between now and path planning in Term 3, I plan to break this MPC in even more ways that I have so far, in order to better grasp what each component is truely doing!
  
  There was an unofficial instruction to see how fast we could get the car to go around the track without driving off.  I managed to reach 80mph with the current weighted values, however the car loses the road at the tires if I bump that to 100mph.
