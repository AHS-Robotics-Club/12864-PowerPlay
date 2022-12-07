package org.firstinspires.ftc.teamcode.auton.ops;//package org.firstinspires.ftc.teamcode.auton.ops;
//
////import com.arcrobotics.ftclib.command.CommandOpMode;
////import com.arcrobotics.ftclib.command.InstantCommand;
////import com.arcrobotics.ftclib.command.ParallelCommandGroup;
////import com.arcrobotics.ftclib.command.WaitCommand;
////import com.arcrobotics.ftclib.command.WaitUntilCommand;
////import com.arcrobotics.ftclib.hardware.motors.Motor;
////import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.DcMotorSimple;
////
////@Autonomous(name = "ParkingAuton")
////public class ParkingAuton extends CommandOpMode {
////
////    private Motor fL, bL, fR, bR;
////
////    @Override
////    public void initialize() {
////
////        fL = new Motor(hardwareMap, "fL");
////        fR = new Motor(hardwareMap, "fR");
////        bL = new Motor(hardwareMap, "bL");
////        bR = new Motor(hardwareMap, "bR");
////
////        fL.motor.setDirection(DcMotor.Direction.REVERSE);
////        bR.motor.setDirection(DcMotor.Direction.REVERSE);
////
////        schedule(
////            new WaitUntilCommand(this::isStarted)
////                .andThen(new ParallelCommandGroup(
////                    new InstantCommand(() -> fL.set(-1)),
////                    new InstantCommand(() -> bL.set(1)),
////                    new InstantCommand(() -> fR.set(1)),
////                    new InstantCommand(() -> bR.set(-1))
////                )).andThen(
////                    new WaitCommand(850))
////                .andThen(new ParallelCommandGroup(
////                    new InstantCommand(() -> fL.stopMotor()),
////                    new InstantCommand(() -> bL.stopMotor()) ,
////                    new InstantCommand(() -> fR.stopMotor()),
////                    new InstantCommand(() -> bR.stopMotor())
////                    )
////                )
////            );
////
////
////
////    }
////}

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.rr.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.rr.MecanumDriveSubsystem;

@Autonomous(name = "ParkingAuton")
public class ParkingAuton extends CommandOpMode{
    private MecanumDriveSubsystem drive;
    private Motor frontLeft, frontRight, backLeft, backRight;

    private Pose2d startPose = new Pose2d(35.0, -62.0, Math.toRadians(0.0));

    @Override
    public void initialize() {
        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        frontLeft.motor.setDirection(DcMotor.Direction.REVERSE);
        frontRight.motor.setDirection(DcMotor.Direction.FORWARD);
        backLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        backRight.motor.setDirection(DcMotor.Direction.REVERSE);

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        Trajectory traj0 = drive.trajectoryBuilder(startPose).forward(30.0).build();
//        Trajectory traj0 = drive.trajectoryBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(15, -40, Math.toRadians(-45)), Math.toRadians(0))
//                .build();

        TrajectoryFollowerCommand autonomous = new TrajectoryFollowerCommand(drive, traj0);

        if(isStopRequested()){
            return;
        }

        schedule(autonomous);
    }
}
