package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.GeneralStatus;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase{
    public PigeonIMU pigeon;
    private double[] xyz;
    private double[] speedxyz;
    private short[] acceleration;

    public boolean autoReady;

    public GyroSubsystem(){
        pigeon = new PigeonIMU(0);
        GeneralStatus status = new GeneralStatus();
        autoReady = true;
        pigeon.getGeneralStatus(status);
        if(status.state == PigeonState.NoComm) {
            System.err.println("Error finding pigeon");
            autoReady = false;
        }
        xyz = new double[3];
        speedxyz = new double[3];
        acceleration = new short[3];
        pigeon.setYaw(0);   //In degrees
    }

    private static GyroSubsystem singleton;
    public static GyroSubsystem getInstance(){
        if(singleton == null) singleton = new GyroSubsystem();
        //Yi is a uhcuhuhajnbiciwinr
        return singleton;
    }

    public void zeroHeading(){
        pigeon.setYaw(0);
    }

    public void setAngle(double angle){
        pigeon.setYaw(angle);
    }

    public double getAbsoluteAngle(){
        double yaw = getYawPitchRole()[0];
        if(yaw < 0){
            return 360 - (Math.abs(yaw) % 360);
        }
        return getYawPitchRole()[0] % 360;
    }

    public double[] getYawPitchRole(){
        double[] YPR = new double[3];
        pigeon.getYawPitchRoll(YPR);
        return YPR;
    }

    public double[] getArray(){
        pigeon.getAccumGyro(xyz);
        return xyz;
    }

    public double getX(){
        pigeon.getAccumGyro(xyz);
        return xyz[0];
    }

    public double getY(){
        pigeon.getAccumGyro(xyz);
        return xyz[1];
    }

    public double getZ(){
        pigeon.getAccumGyro(xyz);
        return xyz[2];
    }

    public double[] getSpeedArray(){
        pigeon.getRawGyro(speedxyz);
        return speedxyz;
    }

    public double getSpeedX(){
        pigeon.getRawGyro(speedxyz);
        return speedxyz[0];
    }

    public double getSpeedY(){
        pigeon.getRawGyro(speedxyz);
        return speedxyz[1];
    }

    public double getSpeedZ(){
        pigeon.getRawGyro(speedxyz);
        return speedxyz[2];
    }

    public double getAccelX(){
        pigeon.getBiasedAccelerometer(acceleration);
        return acceleration[0];
    }

    public double getAccelY(){
        pigeon.getBiasedAccelerometer(acceleration);
        return acceleration[1];
    }

    public double getAccelZ(){
        pigeon.getBiasedAccelerometer(acceleration);
        return acceleration[2];
    }
}