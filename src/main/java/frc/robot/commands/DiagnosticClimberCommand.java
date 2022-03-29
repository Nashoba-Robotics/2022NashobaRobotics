package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PusherSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;
import frc.robot.subsystems.PusherSubsystem.PusherMotor;

public class DiagnosticClimberCommand extends CommandBase {
    SendableChooser<ControlMode> climberControlMode;
    SendableChooser<ControlMode> pusherControlMode;

    public DiagnosticClimberCommand() {
        addRequirements(ClimberSubsystem.getInstance());

        climberControlMode = new SendableChooser<>();
        climberControlMode.setDefaultOption("Percent", ControlMode.PercentOutput);
        climberControlMode.addOption("Velocity", ControlMode.Velocity);
        climberControlMode.addOption("Motion Magic", ControlMode.MotionMagic);
        SmartDashboard.putData("Climber Control Mode", climberControlMode);

        pusherControlMode = new SendableChooser<>();
        pusherControlMode.setDefaultOption("Percent", ControlMode.PercentOutput);
        pusherControlMode.addOption("Velocity", ControlMode.Velocity);
        pusherControlMode.addOption("Motion Magic", ControlMode.MotionMagic);
        SmartDashboard.putData("Pusher Control Mode", pusherControlMode);

        SmartDashboard.putNumber("L Pusher Coeff", 1);
        SmartDashboard.putNumber("R Pusher Coeff", 1);
        SmartDashboard.putNumber("Pusher speed", 0);

        SmartDashboard.putNumber("Pusher F", Constants.Climber.KF_PUSHER);
        SmartDashboard.putNumber("Pusher P", Constants.Climber.KP_PUSHER);
        SmartDashboard.putNumber("Pusher I", Constants.Climber.KI_PUSHER);
        SmartDashboard.putNumber("Pusher D", Constants.Climber.KD_PUSHER);

        SmartDashboard.putNumber("L Pusher Cruise Velocity", Constants.Climber.DEPLOY_PUSH_CRUISE_VELOCITY);
        SmartDashboard.putNumber("L Pusher Acceleration", Constants.Climber.DEPLOY_PUSH_ACCELERATION);

        SmartDashboard.putNumber("R Pusher Cruise Velocity", Constants.Climber.DEPLOY_PUSH_CRUISE_VELOCITY);
        SmartDashboard.putNumber("R Pusher Acceleration", Constants.Climber.DEPLOY_PUSH_ACCELERATION);

        SmartDashboard.putNumber("L Pusher Pos", PusherSubsystem.getInstance().getPosition(PusherMotor.LEFT_PUSHER));
        SmartDashboard.putNumber("R Pusher Pos", PusherSubsystem.getInstance().getPosition(PusherMotor.RIGHT_PUSHER));

        SmartDashboard.putNumber("L Pusher Curr", PusherSubsystem.getInstance().getStatorCurrent(PusherMotor.LEFT_PUSHER));
        SmartDashboard.putNumber("R Pusher Curr", PusherSubsystem.getInstance().getStatorCurrent(PusherMotor.RIGHT_PUSHER));
        
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("L Climber Coeff", 1);
        SmartDashboard.putNumber("R Climber Coeff", 1);
        SmartDashboard.putNumber("Climber speed", 0);

        SmartDashboard.putNumber("Climber F", Constants.Climber.KF_CLIMBER);
        SmartDashboard.putNumber("Climber P", Constants.Climber.KP_CLIMBER);
        SmartDashboard.putNumber("Climber I", Constants.Climber.KI_CLIMBER);
        SmartDashboard.putNumber("Climber D", Constants.Climber.KD_CLIMBER);

        SmartDashboard.putNumber("L Climber Cruise Velocity", Constants.Climber.DEPLOY_LEFT_CRUISE_VELOCITY);
        SmartDashboard.putNumber("L Climber Acceleration", Constants.Climber.DEPLOY_LEFT_ACCELERATION);

        SmartDashboard.putNumber("R Climber Cruise Velocity", Constants.Climber.DEPLOY_RIGHT_CRUISE_VELOCITY);
        SmartDashboard.putNumber("R Climber Acceleration", Constants.Climber.DEPLOY_RIGHT_ACCELERATION);

        SmartDashboard.putNumber("L Climber Pos", ClimberSubsystem.getInstance().getLeftPosition());
        SmartDashboard.putNumber("R Climber Pos", ClimberSubsystem.getInstance().getRightPosition());

        SmartDashboard.putNumber("L Climber Curr", ClimberSubsystem.getInstance().getLeftStatorCurrent());
        SmartDashboard.putNumber("R Climber Curr", ClimberSubsystem.getInstance().getRightStatorCurrent());

        
        SmartDashboard.putNumber("L Pusher Coeff", 1);
        SmartDashboard.putNumber("R Pusher Coeff", 1);
        SmartDashboard.putNumber("Pusher speed", 0);

        SmartDashboard.putNumber("Pusher F", Constants.Climber.KF_PUSHER);
        SmartDashboard.putNumber("Pusher P", Constants.Climber.KP_PUSHER);
        SmartDashboard.putNumber("Pusher I", Constants.Climber.KI_PUSHER);
        SmartDashboard.putNumber("Pusher D", Constants.Climber.KD_PUSHER);

        SmartDashboard.putNumber("L Pusher Cruise Velocity", Constants.Climber.DEPLOY_PUSH_CRUISE_VELOCITY);
        SmartDashboard.putNumber("L Pusher Acceleration", Constants.Climber.DEPLOY_PUSH_ACCELERATION);

        SmartDashboard.putNumber("R Pusher Cruise Velocity", Constants.Climber.DEPLOY_PUSH_CRUISE_VELOCITY);
        SmartDashboard.putNumber("R Pusher Acceleration", Constants.Climber.DEPLOY_PUSH_ACCELERATION);

        SmartDashboard.putNumber("L Pusher Pos", PusherSubsystem.getInstance().getPosition(PusherMotor.LEFT_PUSHER));
        SmartDashboard.putNumber("R Pusher Pos", PusherSubsystem.getInstance().getPosition(PusherMotor.RIGHT_PUSHER));

        SmartDashboard.putNumber("L Pusher Curr", PusherSubsystem.getInstance().getStatorCurrent(PusherMotor.LEFT_PUSHER));
        SmartDashboard.putNumber("R Pusher Curr", PusherSubsystem.getInstance().getStatorCurrent(PusherMotor.RIGHT_PUSHER));
    }

    public void execute() {
        //Uses coefficients to determine if the climbers are run together
        //1 = running   0 = not running
        double lcoeff = SmartDashboard.getNumber("L Climber Coeff", 1);
        double rcoeff = SmartDashboard.getNumber("R Climber Coeff", 1);
        double climbSpeed = SmartDashboard.getNumber("Climber speed", 0);
        ClimberSubsystem.getInstance().setDiagnosticSpeed(ClimberMotor.LEFT_CLIMBER, climbSpeed * lcoeff, climberControlMode.getSelected());
        ClimberSubsystem.getInstance().setDiagnosticSpeed(ClimberMotor.RIGHT_CLIMBER, climbSpeed * rcoeff, climberControlMode.getSelected());

        //Manually Setting Climber PID values
        //Both climbers should have the same PID values
        ClimberSubsystem.getInstance().setF(SmartDashboard.getNumber("Climber F", Constants.Climber.KF_CLIMBER));
        ClimberSubsystem.getInstance().setP(SmartDashboard.getNumber("Climber P", Constants.Climber.KP_CLIMBER));
        ClimberSubsystem.getInstance().setI(SmartDashboard.getNumber("Climber I", Constants.Climber.KI_CLIMBER));
        ClimberSubsystem.getInstance().setD(SmartDashboard.getNumber("Climber D", Constants.Climber.KD_CLIMBER));

        //Sets cruise velocity and acceleration for motion magic
        ClimberSubsystem.getInstance().setCruiseVelocity(ClimberMotor.LEFT_CLIMBER, SmartDashboard.getNumber("L Climber Cruise Velocity", Constants.Climber.DEPLOY_LEFT_CRUISE_VELOCITY));
        ClimberSubsystem.getInstance().setAcceleration(ClimberMotor.LEFT_CLIMBER, SmartDashboard.getNumber("L Climber Acceleration", Constants.Climber.DEPLOY_LEFT_ACCELERATION));

        ClimberSubsystem.getInstance().setCruiseVelocity(ClimberMotor.RIGHT_CLIMBER, SmartDashboard.getNumber("R Climber Cruise Velocity", Constants.Climber.DEPLOY_RIGHT_CRUISE_VELOCITY));
        ClimberSubsystem.getInstance().setAcceleration(ClimberMotor.RIGHT_CLIMBER, SmartDashboard.getNumber("R Climber Acceleration", Constants.Climber.DEPLOY_RIGHT_ACCELERATION));

        //Diagnostic values for encoder position and current for each climber
        SmartDashboard.putNumber("L Climber Pos", ClimberSubsystem.getInstance().getLeftPosition());
        SmartDashboard.putNumber("R Climber Pos", ClimberSubsystem.getInstance().getRightPosition());

        SmartDashboard.putNumber("L Climber Curr", ClimberSubsystem.getInstance().getLeftStatorCurrent());
        SmartDashboard.putNumber("R Climber Curr", ClimberSubsystem.getInstance().getRightStatorCurrent());

        
        double lPushCoeff = SmartDashboard.getNumber("L Climber Coeff", 1);
        double rPushCoeff = SmartDashboard.getNumber("R Climber Coeff", 1);
        double pushSpeed = SmartDashboard.getNumber("Pusher speed", 0);
        PusherSubsystem.getInstance().setDiagnosticSpeed(PusherMotor.LEFT_PUSHER, pushSpeed * lPushCoeff, pusherControlMode.getSelected());
        PusherSubsystem.getInstance().setDiagnosticSpeed(PusherMotor.RIGHT_PUSHER, pushSpeed * rPushCoeff, pusherControlMode.getSelected());

        //Sets Pusher PID values
        PusherSubsystem.getInstance().setF(SmartDashboard.getNumber("Pusher F", Constants.Climber.KF_PUSHER));
        PusherSubsystem.getInstance().setP(SmartDashboard.getNumber("Pusher P", Constants.Climber.KP_PUSHER));
        PusherSubsystem.getInstance().setI(SmartDashboard.getNumber("Pusher I", Constants.Climber.KI_PUSHER));
        PusherSubsystem.getInstance().setD(SmartDashboard.getNumber("Pusher D", Constants.Climber.KD_PUSHER));

        //Sets cruise velocity and acceleration for motion magic
        PusherSubsystem.getInstance().setCruiseVelocity(PusherMotor.LEFT_PUSHER, SmartDashboard.getNumber("L Pusher Cruise Velocity", Constants.Climber.DEPLOY_PUSH_CRUISE_VELOCITY));
        PusherSubsystem.getInstance().setAcceleration(PusherMotor.LEFT_PUSHER, SmartDashboard.getNumber("L Pusher Acceleration", Constants.Climber.DEPLOY_PUSH_ACCELERATION));

        PusherSubsystem.getInstance().setCruiseVelocity(PusherMotor.RIGHT_PUSHER, SmartDashboard.getNumber("R Pusher Cruise Velocity", Constants.Climber.DEPLOY_PUSH_CRUISE_VELOCITY));
        PusherSubsystem.getInstance().setAcceleration(PusherMotor.RIGHT_PUSHER, SmartDashboard.getNumber("R Pusher Acceleration", Constants.Climber.DEPLOY_PUSH_ACCELERATION));

        //Diagnostic values for encoder position and current for each pusher
        SmartDashboard.putNumber("L Pusher Pos", PusherSubsystem.getInstance().getPosition(PusherMotor.LEFT_PUSHER));
        SmartDashboard.putNumber("R Pusher Pos", PusherSubsystem.getInstance().getPosition(PusherMotor.RIGHT_PUSHER));

        SmartDashboard.putNumber("L Pusher Curr", PusherSubsystem.getInstance().getStatorCurrent(PusherMotor.LEFT_PUSHER));
        SmartDashboard.putNumber("R Pusher Curr", PusherSubsystem.getInstance().getStatorCurrent(PusherMotor.RIGHT_PUSHER));
    }
 
    @Override
    public void end(boolean interrupted) {
        ClimberSubsystem.getInstance().stop();
        PusherSubsystem.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
