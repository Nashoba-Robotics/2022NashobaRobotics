package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class OdometryCarpetCompensator extends DifferentialDriveOdometry{
    
    //private double startAngle; // starting angle relative to direction on rug where there is no affect on odometry
    private double angOfResistance;
    private double absAngle; // consistantly updated angle compared to starting angle

    private double lastLeftNU;
    private double lastRightNU;

    // start Angle is relative to the direction on the rug where there is no affect on odometry (0 -> TAU)
    // currAngle is what the gyro is going to read when the auto command starts
    public OdometryCarpetCompensator(double angOfResistance, Rotation2d currAngle){
        super(currAngle);
        lastLeftNU = 0;
        lastRightNU = 0;
        this.angOfResistance = Units.getAbsAngle(angOfResistance);
        absAngle = Units.getAbsAngle(currAngle.getRadians() - angOfResistance);
    }

    public Pose2d updatePose2d(Rotation2d angle, double deltaLeftNU, double deltaRightNU){
        absAngle = Units.getAbsAngle(angle.getRadians() - angOfResistance);

        double leftNU = lastLeftNU + compensateRug(deltaLeftNU);
        double rightNU = lastRightNU + compensateRug(deltaRightNU);

        Pose2d pose = super.update(
            angle,
            Units.NU2Meters(leftNU),
            Units.NU2Meters(rightNU));

        lastLeftNU = leftNU;
        lastRightNU = rightNU;

        return pose;

    }

    public double getAngle(){
        return absAngle;
    }

    private double compensateRug(double delta){
        if(DriverStation.getAlliance() == Alliance.Red){
            return delta > 0 ?
            delta + (delta*Constants.FIELD.K_CARPET_RED/2)*(1-Math.cos(absAngle)):
            delta + (delta*Constants.FIELD.K_CARPET_RED/2)*(1+Math.cos(absAngle));
        }

        return delta > 0 ?
            delta + (delta*Constants.FIELD.K_CARPET_BLUE/2)*(1-Math.cos(absAngle)):
            delta + (delta*Constants.FIELD.K_CARPET_BLUE/2)*(1+Math.cos(absAngle));
        
    }

    // call to reset the odometry position
    // zero heading AFTER calling function
    public void resetPos(Pose2d pos, Rotation2d rotation, double angOfResistance){
        lastLeftNU = 0;
        lastRightNU = 0;
        this.angOfResistance = Units.getAbsAngle(angOfResistance);
        absAngle = Units.getAbsAngle(rotation.getRadians() - angOfResistance);
        super.resetPosition(pos, rotation);
    }

    public void resetPos(Pose2d pos, Rotation2d rotation){
        lastLeftNU = 0;
        lastRightNU = 0;
        absAngle = Units.getAbsAngle(rotation.getRadians() - angOfResistance);
        super.resetPosition(pos, rotation);
    }

    public double getLeftMeters(){
        return Units.NU2Meters(lastLeftNU);
    }

    public double getRightMeters(){
        return Units.NU2Meters(lastRightNU);
    }

    public double getLeftNU(){
        return lastLeftNU;
    }

    public double getRightNU(){
        return lastRightNU;
    }

}
