package frc.robot.PassThroughSystems.IMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;;

public class IMUAdis implements IMUInterface {

    private ADIS16470_IMU adis; 

    public IMUAdis ()
    {
        try {
            adis = new ADIS16470_IMU(); 
        } catch (RuntimeException ex) {
            // TODO: handle exception
            System.out.println("Exception while crating IMU"); 
        }
    }

    @Override
    public double getPitch() {
        // TODO Auto-generated method stub
        return adis.getAngle(IMUAxis.kY); 
    }

    @Override
    public double getRoll() {
        // TODO Auto-generated method stub
        return adis.getAngle(IMUAxis.kX); 
    }

    @Override
    public double getYaw() {
        return adis.getAngle(IMUAxis.kZ); 
    }

    @Override
    public Rotation2d getYawRotation2d() {
        // TODO Auto-generated method stub
        return Rotation2d.fromDegrees(getYaw()); 
    }

    
    @Override
    public double zeroYaw() {
        double previousYaw = getYaw(); 
        adis.setGyroAngle(adis.getYawAxis(), 0.0);
        return previousYaw; 
    }

    @Override
    public double setYaw(double y) {
        double previousYaw = getYaw(); 
        adis.setGyroAngle(adis.getYawAxis(), y);
        return previousYaw; 
    }

    @Override
    public double getTurnRate() {
        return adis.getRate(adis.getYawAxis()); 
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(adis.getAngle(adis.getYawAxis())); 
    }

}