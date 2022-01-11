package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private static LimelightSubsystem instance;
    
    private NetworkTableEntry validTarget;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry ts;
    private NetworkTableEntry tl;
    private NetworkTableEntry tshort;
    private NetworkTableEntry tlong;
    private NetworkTableEntry thor;
    private NetworkTableEntry tvert;
    private NetworkTableEntry pipeline;
    private NetworkTableEntry camTrain;
    private NetworkTableEntry ledMode;

    private LimelightSubsystem() {
        SendableRegistry.setName(this, "Limelight");
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        validTarget = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
        tl = table.getEntry("ts");
        tshort = table.getEntry("ts");
        tlong = table.getEntry("ts");
        thor = table.getEntry("ts");
        tvert = table.getEntry("ts");
        pipeline = table.getEntry("ts");
        camTrain = table.getEntry("ts");
        ledMode = table.getEntry("ledMode");
    }

    public static LimelightSubsystem getInstance() {
        if(instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    //Sets the current limelight pipeline (0-9)
    public void setPipeline(int input){
        pipeline.setDouble(input);
    }

    //Sets camera mode to Vision Processor(0) or driver camera(1)
    public void setCamTrain(int input){
        camTrain.setDouble(input);
    }

    //Has the limelight locked on to a valid target?
    public boolean validTarget(){
        return validTarget.getDouble(0) == 1;
    }

    //What is the pipeline that we are currently using?
    public double getPipe(){
        return pipeline.getDouble(0);
    }

    //What is the vertical sidelength of the rough bounding box? (0 - 320 pixels)
    public double getTvert(){
        return tvert.getDouble(0);
    }

    //What is the horizontal sidelength of the rough bounding box? (0 - 320 pixels)
    public double getThor(){
        return thor.getDouble(0);
    }

    //What is the longer sidelength of the fitted bounding box? (in pixels)
    public double getTlong(){
        return tlong.getDouble(0);
    }

    //What is the shorter sidelength of the fitted bounding box? (in pixels)
    public double getTshort(){
        return tshort.getDouble(0);
    }

    //What is the latency of the limelight? (Your really using this? It isn't useful)
    public double getTl(){
        return tl.getDouble(0);
    }

    //What is the skew/rotation (0 to -90 degrees)
    public double getTs(){
        return ts.getDouble(0);
    }

    //What is the x value of the object from the crosshair that the limelight has locked on to? (-27 to 27 degrees)
    public double getTx() {
        return tx.getDouble(0);
    }

    //what is the y value of the object from the crosshair that the limelight has locked on to? (-20.5 to 20.5 degrees)
    public double getTy() {
        return ty.getDouble(0);
    }

    public double getTa() {
        return ta.getDouble(0);
    }

    public void setLed(double led) {
        ledMode.setDouble(led);    
    }

    public double getDistance(double h2){
        // d = 2 m
        // hieght 1 = 0.915 m
        // hieght 2 = 1.08 cm
        // a1 = 16 deg = 2tau/45
        double h1 = 0.915;
        double a1 = -16*Math.PI/180;
        double a2 = getTy() * Math.PI/180;
        return (h2-h1) / Math.tan(a1+a2);
    }

    public double getDistanceBall(){
        double h1 = 0.915;
        double h2 =  0;
        double a1 = -16*Math.PI/180;
        double a2 = getTy() * Math.PI/180;
        return ((h2-h1) / Math.tan(a1+a2)) * 2.075 - 0.714;
    }
}