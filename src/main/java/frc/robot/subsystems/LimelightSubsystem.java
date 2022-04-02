package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    private static LimelightSubsystem instance;
    
    private NetworkTableEntry intakeValidTarget;
    private NetworkTableEntry intakeTx;
    private NetworkTableEntry intakeTy;
    private NetworkTableEntry intakePipeline;
    private NetworkTableEntry intakeLedMode;

    private NetworkTableEntry shooterValidTarget;
    private NetworkTableEntry shooterTx;
    private NetworkTableEntry shooterTy;
    private NetworkTableEntry shooterPipeline;
    private NetworkTableEntry shooterLedMode;

    private LimelightSubsystem() {
        SendableRegistry.setName(this, "Limelight");
        NetworkTable intake = NetworkTableInstance.getDefault().getTable("limelight-intake");
        intakeValidTarget = intake.getEntry("tv");
        intakeTx = intake.getEntry("tx");
        intakeTy = intake.getEntry("ty");
        intakePipeline = intake.getEntry("pipeline");
        intakeLedMode = intake.getEntry("ledMode");
        NetworkTable shooter = NetworkTableInstance.getDefault().getTable("limelight-shooter");
        shooterValidTarget = shooter.getEntry("tv");
        shooterTx = shooter.getEntry("tx");
        shooterTy = shooter.getEntry("ty");
        shooterPipeline = shooter.getEntry("pipeline");
        shooterLedMode = shooter.getEntry("ledMode");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight Angle", getIntakeTx());
        SmartDashboard.putNumber("Limelight Y", getIntakeTy());
    }

    public static LimelightSubsystem getInstance() {
        if(instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    //Sets the current limelight pipeline (0-9)
    public void setIntakePipeline(int input){
        intakePipeline.setDouble(input);
    }

    //Has the limelight locked on to a valid target?
    public boolean intakeValidTarget(){
        return intakeValidTarget.getDouble(0) == 1;
    }

    //Has the limelight locked on to a valid target?
    public boolean shooterValidTarget(){
        return shooterValidTarget.getDouble(0) == 1;
    }

    //What is the pipeline that we are currently using?
    public double getIntakePipe(){
        return intakePipeline.getDouble(0);
    }

    //What is the pipeline that we are currently using?
    public double getShooterPipe(){
        return shooterPipeline.getDouble(0);
    }


    //What is the x value of the object from the crosshair that the limelight has locked on to? (-27 to 27 degrees)
    public double getIntakeTx() {
        return intakeTx.getDouble(0);
    }

    //What is the x value of the object from the crosshair that the limelight has locked on to? (-27 to 27 degrees)
    public double getShooterTx() {
        return shooterTx.getDouble(0);
    }

     //What is the x value of the object from the crosshair that the limelight has locked on to? (-27 to 27 degrees)
     public double getIntakeTy() {
        return intakeTy.getDouble(0);
    }

    //What is the x value of the object from the crosshair that the limelight has locked on to? (-27 to 27 degrees)
    public double getShooterTy() {
        return shooterTy.getDouble(0);
    }

    public void setIntakeLed(double led) {
        intakeLedMode.setDouble(led);    
    }

    public void setShooterLed(double led) {
        shooterLedMode.setDouble(led);    
    }

    public double getDistanceShooter(){
        double h1 = Constants.Limelight.SHOOTER_HEIGHT;
        double h2 = Constants.Limelight.HUB_HEIGHT;
        double a1 = Constants.Limelight.SHOOTER_ANGLE;
        double a2 = getShooterTy() * Constants.TAU/360; // angle that limelight sees target
        double distance = (h2-h1) / Math.tan(a1+a2);
        return distance;
    }

    public double getDistanceIntake(){
        double h1 = Constants.Limelight.INTAKE_HEIGHT;
        double h2 = Constants.Limelight.BALL_HEIGHT;
        double a1 = Constants.Limelight.INTAKE_ANGLE;
        double a2 = getIntakeTy() * Constants.TAU/360; // angle that limelight sees target
        double distance = (h2-h1) / Math.tan(a1+a2);
        return distance;
    }
}