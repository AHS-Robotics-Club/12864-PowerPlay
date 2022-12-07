package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

public class ClawSubsystem extends SubsystemBase {
    private SimpleServo claw;

    public ClawSubsystem(SimpleServo c){
        claw = c;
    }
    public void close(int deg){
        claw.turnToAngle(deg);
    }
    public void open(int deg){
        claw.turnToAngle(deg);
    }
}
