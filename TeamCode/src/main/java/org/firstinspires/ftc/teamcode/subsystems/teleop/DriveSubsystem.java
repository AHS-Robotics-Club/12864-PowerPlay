package org.firstinspires.ftc.teamcode.subsystems.teleop;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveSubsystem extends SubsystemBase {
    private Motor frontLeft, frontRight, backLeft, backRight;
    private MecanumDrive mecanumDrive;

    public DriveSubsystem(Motor fL, Motor fR, Motor bL, Motor bR, RevIMU imu){
        frontLeft = fL;
        frontRight = fR;
        backLeft = bL;
        backRight = bR;

        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    // Method which runs the subsystem
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        mecanumDrive.driveRobotCentric(-strafeSpeed, -forwardSpeed, -turnSpeed, true);
    }
}
