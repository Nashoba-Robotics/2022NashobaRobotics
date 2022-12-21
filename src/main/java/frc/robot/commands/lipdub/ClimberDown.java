package frc.robot.commands.lipdub;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDown extends CommandBase{

    private final int leftPos = 20_000;
    public final int rightPos = Constants.Climber.DEPLOY_RIGHT_POS;
    
    public ClimberDown(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ClimberSubsystem.getInstance().setPosition(leftPos, rightPos);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(ClimberSubsystem.getInstance().getRightPosition()-rightPos) < 500;
        // || Math.abs(ClimberSubsystem.getInstance().getLeftPosition()-leftPos) < 500;
    }

}
