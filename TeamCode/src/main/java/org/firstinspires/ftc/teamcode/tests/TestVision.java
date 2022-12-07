package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled // TODO: Remove this in order to show op-mode on phone
@Autonomous(name = "TestVision")
public class TestVision extends CommandOpMode {

    // TODO: Declare detector
    // private FFCapstoneDetector capstoneDetector;

    @Override
    public void initialize() {

        // TODO: Initialize detector
        /*
        capstoneDetector = new FFCapstoneDetector(hardwareMap, "bigBrother");
        capstoneDetector.init();
        capstoneDetector.setLowerAndUpperBounds(new Scalar(20, 142, 20), new Scalar(255, 163, 90));
         */

        // TODO: Let FTC Dashboard get camera stream from detector
        // FtcDashboard.getInstance().startCameraStream(capstoneDetector.getCamera(), 30);

        schedule(
            new WaitUntilCommand(this::isStarted).andThen(new RunCommand(() -> {
                // Add telemetry
                /*
                telemetry.addData("Capstone Placement", capstoneDetector.getPlacement());
                telemetry.addData("Lift Pos", capstoneDetector.placementId());
                 */

                telemetry.update();
            }))
        );
    }
}
