package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ZeroClimberSensorsCommand extends CommandBase{
    @Override
    public void initialize() {
        ClimberSubsystem.getInstance().zeroSensors();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
