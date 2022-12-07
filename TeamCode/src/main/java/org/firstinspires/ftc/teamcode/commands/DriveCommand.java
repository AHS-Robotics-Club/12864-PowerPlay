package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private DriveSubsystem drive;
    private DoubleSupplier leftY, leftX, rightX;
    private double mult;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, double mult){

        this.drive = drive;

        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;

        this.mult = mult;

        addRequirements(drive);
    }

    @Override
    public void execute(){
        drive.drive(leftY.getAsDouble() * mult, leftX.getAsDouble() * mult, rightX.getAsDouble() * mult);
    }

}
