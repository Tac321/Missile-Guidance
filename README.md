## Missile-Guidance
Tracking of targets using Proportional Navigation (PNG) on a four fin rocket configuration.

## Simulation Test Cases
This simulation of a guided projectile was ran for impact angle designation targeting and tracking of a moving target under sinusoidal oscillation.  Proportional Navigation (PNG) was used for the missile tracking algorithms, and this model includes a 3-Loop autopilot model. This model can target static and moving targets.

## Illustrations
# Impact Angle Designation
<img src="https://github.com/Tac321/Missile-Guidance/blob/master/Images/ImpactAngleDesig.gif" width="700" />

# Tracking Maneuvering Target
<img src="https://github.com/Tac321/Missile-Guidance/blob/master/Images/ManeuvTargetPursuit.gif" width="700" />

## How to run
# Run code
1) Run Msl_COMPLETE_ADD_ImpactAngle.pde.
2) To turn on the simulation press "g". Feel free to add additional rocket thrust "Rub" using the slider on the right side of the screen.
3) In the beginning of the flight choose the impact angle "XLAMDoffset" in radians,  you wish the projectile to contact the target. (Note: the code cannot handle an impact angle of 90 degrees yet due to singularity of Euler angle sequence. Work is being pursued to mitigate this.

### Note:
The projectile roll was frozen, however the projectile still functions even if it is rolling.
