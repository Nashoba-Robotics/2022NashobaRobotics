package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LedSubsystem.LedStateType;

public class ManualClimberCommand extends CommandBase{
    public ManualClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LedSubsystem.getInstance().setLedStateType(LedStateType.CLIMB);
        LedSubsystem.getInstance().setClimbColor(Constants.Leds.MANUAL_CLIMB);
    }

    @Override
    public void execute() {
        ClimberSubsystem.getInstance().setSpeed(ClimberSubsystem.getInstance().getClimberJoystickValue());
    }

    @Override
    public void end(boolean interrupted) {
        ClimberSubsystem.getInstance().setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
