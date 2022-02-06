package frc.robot.lib;

//used to represent the final motor values that will be sent to the robot (left & right)
public class MotorValues{
    public double left;
    public double right;

    public MotorValues(double left, double right){
        this.left = left;
        this.right = right;
    }
}