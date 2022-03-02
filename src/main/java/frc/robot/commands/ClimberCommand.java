package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;

public class ClimberCommand extends CommandBase {
    boolean finished;

    public ClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        finished = false;
        SmartDashboard.putNumber("L 1", 0);
        // SmartDashboard.putNumber("L 2", 0);
        // SmartDashboard.putNumber("L Rotate", 0);
        SmartDashboard.putNumber("R 1", 0);
        // SmartDashboard.putNumber("R 2", 0);
        // SmartDashboard.putNumber("R Rotate", 0);
    }

    @Override
    public void execute(){
        ClimberSubsystem csi = ClimberSubsystem.getInstance();
        
        if(csi.isCurrentBad()) {
            csi.stop();
            finished = true;
            return;
        }
        csi.setSpeed(ClimberMotor.RIGHT_1, SmartDashboard.getNumber("R 1",0));
        // csi.setSpeed(ClimberMotor.RIGHT_2, SmartDashboard.getNumber("R 2",0));
        // csi.setSpeed(ClimberMotor.RIGHT_ROTATE, SmartDashboard.getNumber("R Rotate",0));

        SmartDashboard.putNumber("R 1 Stator", csi.getStatorCurrent(ClimberMotor.RIGHT_1));
        // SmartDashboard.putNumber("R 2 Stator", csi.getStatorCurrent(ClimberMotor.RIGHT_2));
        // SmartDashboard.putNumber("R Rotate Stator", csi.getStatorCurrent(ClimberMotor.RIGHT_ROTATE));
        SmartDashboard.putNumber("R 1 Supply", csi.getSupplyCurrent(ClimberMotor.RIGHT_1));
        // SmartDashboard.putNumber("R 2 Supply", csi.getSupplyCurrent(ClimberMotor.RIGHT_2));
        // SmartDashboard.putNumber("R Rotate Supply", csi.getSupplyCurrent(ClimberMotor.RIGHT_ROTATE));
        SmartDashboard.putNumber("R 1 Pos", csi.getPosition(ClimberMotor.RIGHT_1));
        // SmartDashboard.putNumber("R 2 Pos", csi.getPosition(ClimberMotor.RIGHT_2));
        // SmartDashboard.putNumber("R Rotate Pos", csi.getPosition(ClimberMotor.RIGHT_ROTATE));

        SmartDashboard.putBoolean("R 1 Switch", csi.getLimitSwitch(ClimberMotor.RIGHT_1));
        // SmartDashboard.putBoolean("R 2 Switch", csi.getLimitSwitch(ClimberMotor.RIGHT_2));

        csi.setSpeed(ClimberMotor.LEFT_1, SmartDashboard.getNumber("L 1",0));
        // csi.setSpeed(ClimberMotor.LEFT_2, SmartDashboard.getNumber("L 2",0));
        // csi.setSpeed(ClimberMotor.LEFT_ROTATE, SmartDashboard.getNumber("L Rotate",0));

        SmartDashboard.putNumber("L 1 Stator", csi.getStatorCurrent(ClimberMotor.LEFT_1));
        // SmartDashboard.putNumber("L 2 Stator", csi.getStatorCurrent(ClimberMotor.LEFT_2));
        // SmartDashboard.putNumber("L Rotate Stator", csi.getStatorCurrent(ClimberMotor.LEFT_ROTATE));
        SmartDashboard.putNumber("L 1 Supply", csi.getSupplyCurrent(ClimberMotor.LEFT_1));
        // SmartDashboard.putNumber("L 2 Supply", csi.getSupplyCurrent(ClimberMotor.LEFT_2));
        // SmartDashboard.putNumber("L Rotate Supply", csi.getSupplyCurrent(ClimberMotor.LEFT_ROTATE));
        SmartDashboard.putNumber("L 1 Pos", csi.getPosition(ClimberMotor.LEFT_1));
        // SmartDashboard.putNumber("L 2 Pos", csi.getPosition(ClimberMotor.LEFT_2));
        // SmartDashboard.putNumber("L Rotate Pos", csi.getPosition(ClimberMotor.LEFT_ROTATE));

        SmartDashboard.putBoolean("L 1 Switch", csi.getLimitSwitch(ClimberMotor.LEFT_1));
        // SmartDashboard.putBoolean("L 2 Switch", csi.getLimitSwitch(ClimberMotor.LEFT_2));
    }

    @Override
    public void end(boolean interrupted){
        ClimberSubsystem.getInstance().stop();
        finished = true;
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}
