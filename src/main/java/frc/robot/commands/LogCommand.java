package frc.robot.commands;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class LogCommand extends CommandBase{
    DataLog log;

    DoubleLogEntry accelerationX;
    DoubleLogEntry accelerationY;

    DoubleLogEntry leftStator;
    DoubleLogEntry leftSupply;

    DoubleLogEntry rightStator;
    DoubleLogEntry rightSupply;

    @Override
    public void initialize() {
        DataLogManager.start();
        log = DataLogManager.getLog();

        accelerationX = new DoubleLogEntry(log, "accelerationX");
        accelerationY = new DoubleLogEntry(log, "accelerationY");

        leftStator = new DoubleLogEntry(log, "LeftStator");
        leftSupply = new DoubleLogEntry(log, "LeftSupply");

        rightStator = new DoubleLogEntry(log, "rightStator");
        rightSupply = new DoubleLogEntry(log, "rightSupply");
        
    }

    @Override
    public void execute() {
        DriveSubsystem.getInstance().setSpeed(1);
        accelerationX.append(GyroSubsystem.getInstance().getAccelX());
        accelerationY.append(GyroSubsystem.getInstance().getAccelY());

        leftStator.append(DriveSubsystem.getInstance().getLeftMotorCurrent());
        leftSupply.append(DriveSubsystem.getInstance().getLeftSupply());

        rightStator.append(DriveSubsystem.getInstance().getRightMotorCurrent());
        rightSupply.append(DriveSubsystem.getInstance().getRightSupply());
    }

    @Override
    public void end(boolean interrupted) {
        DriveSubsystem.getInstance().setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
