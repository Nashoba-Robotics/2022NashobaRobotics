package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

//Climbing Command
public class RetractClimberCommand extends CommandBase {
    private double lPos;
    private double rPos;
    public RetractClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ClimberSubsystem.getInstance().undeployClimber();
    }

    @Override
    public boolean isFinished() {
        lPos = ClimberSubsystem.getInstance().getLeftPosition();
        rPos = ClimberSubsystem.getInstance().getRightPosition();
        return Math.abs(lPos - Constants.Climber.RETRACT_LEFT_POS) <= Constants.Climber.CLIMB_DEADZONE||
               Math.abs(rPos - Constants.Climber.RETRACT_RIGHT_POS) <= Constants.Climber.CLIMB_DEADZONE;
    }
}
