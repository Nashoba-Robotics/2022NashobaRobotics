package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

//Yi, your command sucks so much that I actually have to fix this command
public class SetStartAngleCommand extends CommandBase{
    double startAngle;
    public SetStartAngleCommand(double startAngle){
        this.startAngle = 
        DriverStation.getAlliance() == Alliance.Blue ?
        startAngle :
        startAngle + Constants.TAU / 2;
    }

    @Override
    public void execute() {
        DriveSubsystem.getInstance().setStartAngle(startAngle);
    }

    @Override
    public boolean isFinished() {
        return startAngle == startAngle;
    }
}
