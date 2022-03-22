package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.Units;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.lib.ColorDetection.BallColor;

public class RunIntakeCommand extends CommandBase {

    private BallColor allianceColor;

    private long pukeMillis;
    private long shootMillis;

    private boolean finishedPuking;
    private boolean finishedShooting;

    public RunIntakeCommand() {
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Intake On?", true);
        SmartDashboard.putNumber("Balls", 0);

        allianceColor = Units.alliance2BallColor(DriverStation.getAlliance());
        finishedPuking = true;
        finishedShooting = true;
    }

    @Override
    public void execute() {
        boolean ball1 = RobotContainer.getSensor2(); //sensor closer to shooter
        boolean ball2 = ball1 && RobotContainer.getSensor1(); //sensor closer to intake

        finishedPuking = finishedPuking ?
        true :
        System.currentTimeMillis() - pukeMillis >= 500;

        finishedShooting = finishedShooting ?
        true :
        System.currentTimeMillis() - shootMillis >= 500;

        if(ColorSensorSubsystem.getInstance().getBall() != Units.oppositeBallColor(allianceColor)
        && finishedPuking
        && finishedShooting){
            IntakeSubsystem.getInstance().set(ball2 ? -0.2 : Constants.Intake.INTAKE_SPEED);
            GrabberSubsystem.getInstance().set(ball2 ? 0 : Constants.Intake.GRABBER_SPEED);
            LoaderSubsystem.getInstance().set(ball1 ? 0 : Constants.Intake.LOADER_SPEED);
        } else if(ball1 && finishedShooting){
            IntakeSubsystem.getInstance().set(-0.3);
            GrabberSubsystem.getInstance().set(-0.3);
            LoaderSubsystem.getInstance().set(ball1 ? 0 : Constants.Intake.LOADER_SPEED);
            finishedPuking = false;
            pukeMillis = System.currentTimeMillis();
        } else {
            IntakeSubsystem.getInstance().set(Constants.Intake.INTAKE_SPEED);
            GrabberSubsystem.getInstance().set(Constants.Intake.GRABBER_SPEED);
            LoaderSubsystem.getInstance().set(Constants.Intake.LOADER_SPEED);
            CommandScheduler.getInstance().schedule(new ShootSpeedCommand(0.1, 500));
            finishedShooting = false;
            shootMillis = System.currentTimeMillis(); 
        }

        int balls = (ball1 ? 1 : 0) + (ball2 ? 1 : 0);
        SmartDashboard.putNumber("Balls", balls);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
        GrabberSubsystem.getInstance().stop();
        LoaderSubsystem.getInstance().stop();
        SmartDashboard.putBoolean("Intake On?", false);
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}  
