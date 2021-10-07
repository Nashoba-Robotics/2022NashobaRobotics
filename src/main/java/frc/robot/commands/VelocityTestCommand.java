package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.Units;
import frc.robot.subsystems.DriveSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VelocityTestCommand extends CommandBase {

    public VelocityTestCommand(){
        addRequirements(DriveSubsystem.getInstance());
        SmartDashboard.putNumber("Movement%", 0);
        SmartDashboard.putNumber("Turning%", 0);
        SmartDashboard.putNumber("Vel-mode", 0);
    }

    @Override
    public void execute(){
        double movement = SmartDashboard.getNumber("Movement%", 0);
        double turning = SmartDashboard.getNumber("Turning%", 0);
        double[] motorInputs = JoystickProcessing.processJoysticks(movement, turning);
        double velMode = SmartDashboard.getNumber("Vel-mode", 0);
        if(velMode != 0) {
            double leftVel = Units.percent2Velocity(motorInputs[0]);
            double rightVel = Units.percent2Velocity(motorInputs[1]);
            SmartDashboard.putNumber("L-percent", leftVel);
            SmartDashboard.putNumber("R-percent", rightVel);
            DriveSubsystem.getInstance().setSpeed(-leftVel, -rightVel, ControlMode.Velocity);
        } else {
            double leftVel = motorInputs[0];
            double rightVel = motorInputs[1];
            DriveSubsystem.getInstance().setSpeed(-leftVel, -rightVel, ControlMode.PercentOutput);
        }
        SmartDashboard.putNumber("L-velocity", DriveSubsystem.getInstance().getLeftMotorVelocity());
        SmartDashboard.putNumber("R-velocity", DriveSubsystem.getInstance().getRightMotorVelocity());

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}