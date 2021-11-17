package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Simple debugging inteface for testing the robot at 
// specific set inputs (instead of joysticks, which are
// hard to control precisely)
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