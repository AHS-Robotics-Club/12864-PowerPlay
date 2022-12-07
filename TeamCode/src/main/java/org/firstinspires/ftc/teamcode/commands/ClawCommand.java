package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase {
    private ClawSubsystem subsystem;
    private int closeDeg, openDeg;

    public ClawCommand(ClawSubsystem sub, int close, int open){
        subsystem = sub;
        closeDeg = close;
        openDeg = open;
    }

    @Override
    public void initialize(){
        subsystem.close(closeDeg);
    }
    public void end(boolean interrupted){
        subsystem.open(openDeg);
    }
}
