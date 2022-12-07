//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.hardware.RevIMU;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.MotorGroupTemp;
//import org.firstinspires.ftc.teamcode.commands.TeleOp.DriveCommand;
//import org.firstinspires.ftc.teamcode.subsystems.teleop.DriveSubsystem;
//
//@TeleOp(name = "Main")
//public class MainTeleOp extends CommandOpMode {
//    // MOTORS
//    private Motor fL, fR, bL, bR;
//    private MotorGroupTemp left, right;
//
//    // EXTRAS
//    private GamepadEx gPad1;
//
//    private DriveSubsystem driveSubsystem;
//    private DriveCommand driveCommand;
//
//    @Override
//    public void initialize() {
//        // Initializing Motors
//        fL = new Motor(hardwareMap, "fL");
//        fR = new Motor(hardwareMap, "fR");
//        bL = new Motor(hardwareMap, "bL");
//        bR = new Motor(hardwareMap, "bR");
//
//        fL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        left = new MotorGroupTemp(fL, bL);
//        right = new MotorGroupTemp(fR, bR);
//        right.setInverted(true);
//
//        // Initializing Extras
//        gPad1 = new GamepadEx(gamepad1);
//
//        driveSubsystem = new DriveSubsystem(left, right);
//        driveCommand = new DriveCommand(driveSubsystem, gPad1::getLeftY, gPad1::getRightX);
//
//        register(driveSubsystem);
//        driveSubsystem.setDefaultCommand(driveCommand);
//
//    }
//}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "ZOOM")
public class MainTeleOp extends CommandOpMode {

//    private Motor frontLeft, backLeft, frontRight, backRight;
    private GamepadEx gpad;
    private FtcDashboard dashboard;
//
//    private SimpleServo leftClaw, rightClaw;
//    private Motor elevator;

//    private DriveSubsystem driveS;
//    private DriveCommand driveC;

    private SimpleServo claw;

    private ClawSubsystem clawSub;
    private ClawCommand clawCom;

    @Override
    public void initialize() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        frontLeft = new Motor(hardwareMap, "fL");
//        frontRight = new Motor(hardwareMap, "fR");
//        backLeft = new Motor(hardwareMap, "bL");
//        backRight = new Motor(hardwareMap, "bR");
//
//        leftClaw = new SimpleServo(hardwareMap, "lClaw", -45, 45);
//        rightClaw = new SimpleServo(hardwareMap, "rClaw", -45, 45);

//        elevator = new Motor(hardwareMap, "elev");

        claw = new SimpleServo(hardwareMap, "claw", -30, 180);

        gpad = new GamepadEx(gamepad1);

        clawSub = new ClawSubsystem(claw);
        clawCom = new ClawCommand(clawSub, 35, -3);

//        frontLeft.motor.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.motor.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.motor.setDirection(DcMotor.Direction.REVERSE);
//        backRight.motor.setDirection(DcMotor.Direction.REVERSE);
//
//        frontLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//
//        driveS = new DriveSubsystem(frontLeft, frontRight, backLeft, backRight);
//        driveC = new DriveCommand(driveS, gpad::getLeftX, gpad::getLeftY, gpad::getRightX, 1.0);
//        gpad.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(
//                new InstantCommand(() -> leftClaw.turnToAngle( -20))
//                    .andThen(new InstantCommand(() -> rightClaw.turnToAngle(20))),
//                new InstantCommand(() -> leftClaw.turnToAngle( 0))
//                        .andThen(new InstantCommand(() -> rightClaw.turnToAngle(0)))
//        );

//        gpad.getGamepadButton(GamepadKeys.Button.DPAD_UP).toggleWhenActive(
//                new InstantCommand(() -> elevator.set(0.5)), new InstantCommand(() -> elevator.set(0)));
//
//        gpad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).toggleWhenActive(
//                new InstantCommand(() -> elevator.set(-0.5)), new InstantCommand(() -> elevator.set(0)));

////
//        register(driveS);
//        driveS.setDefaultCommand(driveC);

        gpad.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(clawCom);

        schedule(new RunCommand(() ->{
            telemetry.addData("claw: ", claw.getAngle());
            telemetry.update();
        }));

    }
}
