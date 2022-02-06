package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//used to break the turret
public class TurretCommand extends CommandBase {
    public TurretCommand(){
        addRequirements(TurretSubsystem.getInstance());
        SmartDashboard.putNumber("Turret%", 0);
    }

    @Override
    public void initialize() {
        TurretSubsystem.getInstance().setVelocity(0);
    }

    @Override
    public void execute(){
        double vel = SmartDashboard.getNumber("Turret%", 0);
        TurretSubsystem.getInstance().setVelocity(vel);
    }

    @Override
    public void end(boolean interrupted) {
        TurretSubsystem.getInstance().setVelocity(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}