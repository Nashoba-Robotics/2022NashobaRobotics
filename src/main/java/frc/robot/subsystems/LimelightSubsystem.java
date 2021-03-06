package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    private static LimelightSubsystem instance;

    private NetworkTableEntry shooterValidTarget;
    private NetworkTableEntry shooterTx;
    private NetworkTableEntry shooterTy;
    private NetworkTableEntry shooterPipeline;
    private NetworkTableEntry shooterLedMode;

    private double currTx;
    private double currTy;

    private Timer timoutTimer;

    @Override
    public void periodic() {
        if(shooterValidTarget()){
            currTx = shooterTx.getDouble(0);
            currTy = shooterTy.getDouble(0);
            timoutTimer.reset();
        } else if(timoutTimer.get() > 0.5){
            currTx = 0;
            currTy = 0;
        }

        SmartDashboard.putNumber("curr tx", currTx);
    }

    private LimelightSubsystem() {
        SendableRegistry.setName(this, "Limelight");
        NetworkTable shooter = NetworkTableInstance.getDefault().getTable("limelight-shooter");
        shooterValidTarget = shooter.getEntry("tv");
        shooterTx = shooter.getEntry("tx");
        shooterTy = shooter.getEntry("ty");
        shooterPipeline = shooter.getEntry("pipeline");
        shooterLedMode = shooter.getEntry("ledMode");

        timoutTimer = new Timer();
        timoutTimer.start();
    }

    public static LimelightSubsystem getInstance() {
        if(instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    //Sets the current limelight pipeline (0-9)
    public void setShooterPipeline(int input){
        shooterPipeline.setDouble(input);
    }

    //Has the limelight locked on to a valid target?
    public boolean shooterValidTarget(){
        return shooterValidTarget.getDouble(0) == 1;
    }


    //What is the pipeline that we are currently using?
    public double getShooterPipe(){
        return shooterPipeline.getDouble(0);
    }

    //What is the x value of the object from the crosshair that the limelight has locked on to? (-27 to 27 degrees)
    public double getShooterTx() {
        return currTx;
    }

    //What is the x value of the object from the crosshair that the limelight has locked on to? (-27 to 27 degrees)
    public double getShooterTy() {
        return currTy;
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
}