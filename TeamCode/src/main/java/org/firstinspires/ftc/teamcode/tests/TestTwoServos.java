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
@Autonomous(name = "TestTwoServos")
public class TestTwoServos extends CommandOpMode {
    private SimpleServo testServo_L;
    private SimpleServo testServo_R;

    @Override
    public void initialize() {
        testServo_L = new SimpleServo(hardwareMap, "servoL", 0, 180);
        testServo_R = new SimpleServo(hardwareMap, "servoR", 0, 180);

        schedule(
                new WaitUntilCommand(this::isStarted)
                        .andThen(
                                new InstantCommand(() -> testServo_L.turnToAngle(180))
                        )
        );
        schedule(
                new WaitUntilCommand(this::isStarted)
                        .andThen(
                                new InstantCommand(() -> testServo_R.turnToAngle(180))
                        )
        );
    }
}