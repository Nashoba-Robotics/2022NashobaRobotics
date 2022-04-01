package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.Constants;

public class OdometryCarpetCompensator extends DifferentialDriveOdometry{
    
    //private double startAngle; // starting angle relative to direction on rug where there is no affect on odometry
    private double angOfResistance;
    private double absAngle; // consistantly updated angle compared to starting angle

    private double lastLeftMeters;
    private double lastRightMeters;

    // start Angle is relative to the direction on the rug where there is no affect on odometry (0 -> TAU)
    // currAngle is what the gyro is going to read when the auto command starts
    public OdometryCarpetCompensator(double angOfResistance, Rotation2d currAngle){
        super(currAngle);
        lastLeftMeters = 0;
        lastRightMeters = 0;
        this.angOfResistance = Units.getAbsAngle(angOfResistance);
        absAngle = Units.getAbsAngle(currAngle.getRadians() - angOfResistance);
    }

    public Pose2d updatePose2d(Rotation2d angle, double deltaLeftMeters, double deltaRightMeters){

        absAngle = Units.getAbsAngle(angle.getRadians() - angOfResistance);

        double leftPos = lastLeftMeters + compensateRug(deltaLeftMeters);
        double rightPos = lastRightMeters + compensateRug(deltaRightMeters);

        Pose2d pose = super.update(
            angle,
            leftPos,
            rightPos);

        lastLeftMeters = leftPos;
        lastRightMeters = rightPos;

        return pose;
    }

    public double getAngle(){
        return absAngle;
    }

    private double compensateRug(double deltaMeters){
        return deltaMeters > 0 ?
        deltaMeters+(deltaMeters*Constants.FIELD.K_CARPET/2)*(1-Math.cos(absAngle)):
        deltaMeters+(deltaMeters*Constants.FIELD.K_CARPET/2)*(1+Math.cos(absAngle));
        
    }

    // call to reset the odometry position
    // zero heading AFTER calling function
    public void resetPos(Pose2d pos, Rotation2d rotation, double angOfResistance){
        lastLeftMeters = 0;
        lastRightMeters = 0;
        this.angOfResistance = Units.getAbsAngle(angOfResistance);
        absAngle = Units.getAbsAngle(rotation.getRadians() - angOfResistance);
        super.resetPosition(pos, rotation);
    }

    public void resetPos(Pose2d pos, Rotation2d rotation){
        lastLeftMeters = 0;
        lastRightMeters = 0;
        absAngle = Units.getAbsAngle(rotation.getRadians() - angOfResistance);
        super.resetPosition(pos, rotation);
    }

    public double getLeftMeters(){
        return lastLeftMeters;
    }

    public double getRightMeters(){
        return lastRightMeters;
    }

}
