package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSolenoidSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ActuateIntakeCommand extends CommandBase {
    private boolean state;

    public ActuateIntakeCommand(boolean state) {
        addRequirements(IntakeSubsystem.getInstance());
        this.state = state;
    }

    @Override
    public void initialize() {
        IntakeSolenoidSubsystem.getInstance().setState(state);
    }
    
    @Override
    public boolean isFinished() {
       return true;
    }
}  
