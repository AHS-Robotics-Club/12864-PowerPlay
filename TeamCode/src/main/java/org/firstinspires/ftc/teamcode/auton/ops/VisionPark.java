package org.firstinspires.ftc.teamcode.auton.ops;//package org.firstinspires.ftc.teamcode.auton.ops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auton.vision.ConeDetector;
import org.firstinspires.ftc.teamcode.commands.rr.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.rr.MecanumDriveSubsystem;

@Autonomous(name = "Meet2_Vision")
public class VisionPark extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private Motor frontLeft, frontRight, backLeft, backRight;

    int color;

    private ConeDetector coneDetector;

    private Pose2d startPose = new Pose2d(35.0, -62.0, Math.toRadians(0.0));

    @Override
    public void initialize() {

        coneDetector = new ConeDetector(hardwareMap, "cam");
        coneDetector.init();

        TrajectoryFollowerCommand autonomous;

        FtcDashboard.getInstance().startCameraStream(coneDetector.getCamera(), 30);

        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        frontLeft.motor.setDirection(DcMotor.Direction.REVERSE);
        frontRight.motor.setDirection(DcMotor.Direction.FORWARD);
        backLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        backRight.motor.setDirection(DcMotor.Direction.REVERSE);

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        Trajectory lot1 = drive.trajectoryBuilder(startPose).strafeLeft(15.0).forward(15).build();
        Trajectory lot2 = drive.trajectoryBuilder(startPose).forward(15.0).build();
        Trajectory lot3 = drive.trajectoryBuilder(startPose).strafeRight(15.0).forward(15.0).build();

        color = coneDetector.getColor();

        if(color == 1){
            autonomous = new TrajectoryFollowerCommand(drive, lot1);
        }else if(color == 2) {
            autonomous = new TrajectoryFollowerCommand(drive, lot2);
        }else if(color == 3){
            autonomous = new TrajectoryFollowerCommand(drive, lot3);
        }else{
            autonomous = new TrajectoryFollowerCommand(drive, lot2);
        }

        if(isStopRequested()){
            return;
        }

        schedule(new WaitCommand(500)
                .andThen(new RunCommand(() -> {
                    telemetry.addData("Cone Park: ", color);
                    telemetry.update();
                }))
//                .andThen(autonomous)
        );
    }
}
