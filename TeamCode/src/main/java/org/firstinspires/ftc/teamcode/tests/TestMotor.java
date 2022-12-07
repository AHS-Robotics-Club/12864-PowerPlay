package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Config
@Disabled // TODO: Remove this in order to show op-mode on phone
@Autonomous(name = "TestMotor")
public class TestMotor extends CommandOpMode {

    // TODO: Declare motor/subsystem
    private Motor m;

    // TODO: Create config variables for FTC Dashboard
    public static double POWER = 0.5;

    @Override
    public void initialize() {

        // TODO: Initialize motor/subsystem
        m = new Motor(hardwareMap, "exampleMotor");

        schedule(
                new WaitUntilCommand(this::isStarted)
                    .andThen(new InstantCommand(() -> m.set(POWER))) // TODO: Modify here to use proper motor/subsystem
                    .andThen(new RunCommand(() -> {
                        // Add telemetry
                        telemetry.addData("Motor Max RPM", m.getMaxRPM());
                        telemetry.addData("Motor Corrected Velo", m.getCorrectedVelocity());

                        telemetry.update();
                    }))
        );

    }
}
