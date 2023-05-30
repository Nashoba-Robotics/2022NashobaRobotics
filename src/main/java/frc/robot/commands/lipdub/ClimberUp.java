package frc.robot.commands.lipdub;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUp extends CommandBase{
        
    public final int leftPos = Constants.Climber.DEPLOY_LEFT_POS;
    private final int rightPos = 20_000;

    public ClimberUp(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ClimberSubsystem.getInstance().setPosition(leftPos, rightPos);
    }

    @Override
    public boolean isFinished() {
        return 
        //Math.abs(ClimberSubsystem.getInstance().getRightPosition()-rightPos) < 500
        false
        || Math.abs(ClimberSubsystem.getInstance().getLeftPosition()-leftPos) < 500;
    }

}
