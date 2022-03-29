package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;

public class ReleaseClimberCommand extends CommandBase{
    private double lPos;
    private double rPos;
    
    public ReleaseClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void execute() {
        lPos = ClimberSubsystem.getInstance().getLeftPosition();
        rPos = ClimberSubsystem.getInstance().getRightPosition();
        ClimberSubsystem.getInstance().releaseClimber();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(lPos - Climber.RELEASE_DEADZONE) >= Climber.RELEASE_LEFT_POS ||
               Math.abs(rPos - Climber.RELEASE_DEADZONE) >= Climber.RELEASE_RIGHT_POS;
    }
}
