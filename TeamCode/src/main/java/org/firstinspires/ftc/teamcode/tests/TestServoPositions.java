package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "ServoPositions")
public class TestServoPositions extends CommandOpMode {
    private SimpleServo servo_L;
    private SimpleServo servo_R;

    @Override
    public void initialize() {
        servo_L = new SimpleServo(hardwareMap, "servoL", 0, 180);
        servo_R = new SimpleServo(hardwareMap, "servoR", 0, -180);

        schedule(
                new WaitUntilCommand(this::isStarted)
                .andThen(
                        new InstantCommand(() -> servo_L.turnToAngle(90))
                        .alongWith(
                                new InstantCommand(() -> servo_R.turnToAngle(-90))
                                .withTimeout(1000)
                                        .andThen(
                                                new InstantCommand(() -> servo_L.turnToAngle(0))
                                                        .alongWith(
                                                                new InstantCommand(() -> servo_R.turnToAngle(0))
                                                        )
                                        )
                        )
                )
        );

    }
}
