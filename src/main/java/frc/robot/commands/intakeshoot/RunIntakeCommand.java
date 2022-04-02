package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

public class RunIntakeCommand extends CommandBase {

    // private BallColor allianceColor;

    // private long pukeMillis;
    // private long shootMillis;

    // private boolean finishedPuking;
    // private boolean finishedShooting;

    // private RenameThisLater thing;

    // private static boolean colorRejection = false;

    // private boolean grabberChange;

    // private boolean lastStateBall2;

    public RunIntakeCommand() {
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Intake On?", true);
        SmartDashboard.putNumber("Balls", 0);

        // allianceColor = Units.alliance2BallColor(DriverStation.getAlliance());
        // finishedPuking = true;
        // finishedShooting = true;

        // grabberChange = false;
        // lastStateBall2 = false;

        // thing = new RenameThisLater(0, 0, 0.3);
    }

    @Override
    public void execute() {
        boolean ball1 = RobotContainer.getSensor2(); //sensor closer to shooter
        boolean ball2 = ball1 && RobotContainer.getSensor1(); //sensor closer to intake

        // finishedPuking = finishedPuking ?
        // true :
        // System.currentTimeMillis() - pukeMillis >= Constants.Intake.COLOR_REJECTION_PUKE_TIME;

        // finishedShooting = finishedShooting ?
        // true :
        // System.currentTimeMillis() - shootMillis >= Constants.Intake.COLOR_REJECTION_SHOOT_TIME;

        // if(finishedShooting) CannonSubsystem.getInstance().set(0);

            // if(ColorSensorSubsystem.getInstance().getBall() != Units.oppositeBallColor(allianceColor)   //Normal intake
            // && finishedPuking
            // && finishedShooting){
            //     IntakeSubsystem.getInstance().set(ball2 ? -0.2 : Constants.Intake.INTAKE_SPEED);
            //     GrabberSubsystem.getInstance().set(ball2 ? 0 : Constants.Intake.GRABBER_SPEED);
            //     LoaderSubsystem.getInstance().set(ball1 ? 0 : Constants.Intake.LOADER_SPEED);
            // } else if(ball1 && finishedShooting){   //Puking out back
            //     IntakeSubsystem.getInstance().set(-0.3);
            //     GrabberSubsystem.getInstance().set(-0.2);
            //     LoaderSubsystem.getInstance().set(ball1 ? 0 : Constants.Intake.LOADER_SPEED);
            //     if(finishedPuking) pukeMillis = System.currentTimeMillis();
            //     finishedPuking = false;
            // } else {    //Puking out front
            //     IntakeSubsystem.getInstance().set(Constants.Intake.INTAKE_SPEED);
            //     GrabberSubsystem.getInstance().set(Constants.Intake.GRABBER_SPEED);
            //     LoaderSubsystem.getInstance().set(Constants.Intake.LOADER_SPEED);
            //     CannonSubsystem.getInstance().set(0.25);
            //     if(finishedShooting || ColorSensorSubsystem.getInstance().getBall() == Units.oppositeBallColor(allianceColor)) shootMillis = System.currentTimeMillis();
            //     finishedShooting = false;
            // }
            IntakeSubsystem.getInstance().set(ball2 ? -0.2 : Constants.Intake.INTAKE_SPEED);
            GrabberSubsystem.getInstance().set(ball2 ? 0 : Constants.Intake.GRABBER_SPEED);
            LoaderSubsystem.getInstance().set(ball1 ? 0 : Constants.Intake.LOADER_SPEED);
        

        // if(ball2 && !lastStateBall2){
        //     grabberChange = true;
        //     lastStateBall2 = true;
        // }
        // if(grabberChange){
        //     thing.restart(0, 0.3);
        //     grabberChange = false;
        // }

        // if(!ball2){
        //     lastStateBall2 = false;
        // }

        int balls = (ball1 ? 1 : 0) + (ball2 ? 1 : 0);
        SmartDashboard.putNumber("Balls", balls);
        // SmartDashboard.putString("Ball Color", ColorSensorSubsystem.getInstance().getBall().toString());
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
        GrabberSubsystem.getInstance().stop();
        LoaderSubsystem.getInstance().stop();
        CannonSubsystem.getInstance().set(0);
        SmartDashboard.putBoolean("Intake On?", false);
    }

    @Override
    public boolean isFinished() {
       return false;
    }

    public static void setColorRejection(boolean bool){
        // colorRejection = bool;
    }

    // public static boolean getColorRejection(){
    //     // return colorRejection;
    // }
}  
