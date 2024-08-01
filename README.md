# Chaos
## Team 997 Outreach Robot
Updated the Chaos robot code from the older 2023 codebase into the current 2024 code.
Each year the code for the robot will need to be updated and this would be a good opportunity for new programming students to learn the framework.

## Details
* Simple Skid-Steer/Tank drivetrain with 4 CIM motors and 4 VictorSPX motor controllers.  2 on left and 2 on right.
* Single wheel ball launcher
  * Ball Indexer with one motor on a VictorSPX controller used to pull a ball from a hand filled hopper to the launcher wheel.
  * Launcher wheel with over-geared wheel being drived by NEO motor on SparkMax Motor Controller.  Needs to be spun up before fed a ball from the indexer.
  * Ball Hood on the ball path with a large geared motor using a TalonSRX and a quadrature encoder to feed back the hood position.  Also a lower limit switch for reset.
