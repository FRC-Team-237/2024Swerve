package frc.robot.PassThroughSystems.IMU;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IMUInterface {
    double getPitch();

    double getRoll();

    double getYaw();

    double zeroYaw();

    double setYaw(double y);

    double getTurnRate();
    
    Rotation2d getHeading();
}
