# README
Contains Raider of the ARC's work-in-progress naming conventions for FTC

# Naming Conventions
## Command-Based Stuff
### Subsystem
- End variable w/ capital S
### Command
- End with _Com
## Motors
  - Drivetrain Motors
    - Mecanum Drivetrain
      - Must be named like such based on corresponding motor.
        - frontLeft
        - frontRight
        - backLeft
        - backRight
    - Other Drivetrains
      - Will be dealt with case-by-case.
  - Other Motors
    - Must begin with the lowercase m.
## Servos
- Must begin with lowercase s.
## Gamepads
- One Gamepad: gPad
- Two Gamepads: gPad1, gPad2
## ElapsedTime
- Must begin with lowercase t and then a number which ascends starting with 0.
## Camera-Related
- All camera-related variables must start with lowercase c or camera.
## Roadrunner
- Pose: Must start with lowercase p
- Trajectories: Must start with traj and always end with the number trajectory it is, starting from one.
  - Starts at one because startPose is traj0 technically.