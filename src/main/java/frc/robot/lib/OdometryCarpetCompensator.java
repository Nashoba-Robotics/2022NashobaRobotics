package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.Constants;

public class OdometryCarpetCompensator extends DifferentialDriveOdometry{
    
    private double startAngle; // starting angle relative to direction on rug where there is no affect on odometry
    private double absAngle; // consistantly updated angle compared to starting angle

    private double lastLeftPos = 0;
    private double lastRightPos = 0;

    // start Angle is relative to the direction on the rug where there is no affect on odometry (0 -> TAU)
    // currAngle is what the gyro is going to read when the auto command starts
    public OdometryCarpetCompensator(double startAngle, Rotation2d currAngle){
        super(currAngle);
        this.startAngle = Units.getAbsAngle(startAngle);
        absAngle = Units.getAbsAngle(this.startAngle + currAngle.getRadians());
    }


    public double getStartAngle(){
        return startAngle;
    }

    public void setStartAngle(double startAngle){
        this.startAngle = Units.getAbsAngle(startAngle);
        absAngle = startAngle;
    }

    public Pose2d updatePose2d(Rotation2d angle, double deltaLeftMeters, double deltaRightMeters){

        double leftPos = lastLeftPos + compensateRug(deltaLeftMeters);
        double rightPos = lastRightPos + compensateRug(deltaRightMeters);

        Pose2d pose = super.update(
            angle,
            leftPos,
            rightPos);

        lastLeftPos = leftPos;
        lastRightPos = rightPos;

        return pose;
    }

    public double getLeftMeters(){
        return lastLeftPos;
    }

    public double getRightMeters(){
        return lastRightPos;
    }

    //radians
    public void updateAngle(double angle){
        // double nextAng = Units.getAbsAngle(startAngle + angle);
        absAngle = Units.getAbsAngle(startAngle + angle);
    }

    public double getAngle(){
        return absAngle;
    }

    private double compensateRug(double deltaMeters){
        return deltaMeters > 0 ? 
            deltaMeters * (1 + Math.sin(absAngle) * Constants.K_CARPET):
            deltaMeters * (1 + Math.sin(absAngle) * Constants.K_CARPET * -1);
        
    }

    // call to reset the odometry position
    // zero heading AFTER calling function
    public void resetPos(Pose2d pos, Rotation2d rotation){
        absAngle = Units.getAbsAngle(startAngle + rotation.getRadians());
        lastLeftPos = 0;
        lastRightPos = 0;
        super.resetPosition(pos, rotation);
        lastLeftPos = 0;
        lastRightPos = 0;
    }

}
