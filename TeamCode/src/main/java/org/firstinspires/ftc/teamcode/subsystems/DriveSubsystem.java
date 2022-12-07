package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class DriveSubsystem extends SubsystemBase {
    private Motor fL, fR, bL, bR;
    private MecanumDrive mDrive;
    private SampleMecanumDrive drive;
    private int mode;

    public DriveSubsystem(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight){
        fL = frontLeft;
        fR = frontRight;
        bL = backLeft;
        bR = backRight;

        mode = 0;

        mDrive = new MecanumDrive(fL, fR, bL, bR);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed){
        mDrive.driveRobotCentric(-strafeSpeed, -forwardSpeed, -turnSpeed, true);
    }
}
