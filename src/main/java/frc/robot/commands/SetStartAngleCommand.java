package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//Ben, your code sucks so much that I have to make this command
public class SetStartAngleCommand extends CommandBase{
    double startAngle;
    public SetStartAngleCommand(double startAngle){
        this.startAngle = startAngle;
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
