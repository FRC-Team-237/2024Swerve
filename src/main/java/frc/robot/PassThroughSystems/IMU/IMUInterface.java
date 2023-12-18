package frc.robot.PassThroughSystems.IMU;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This interface lists all methods that must be implemented by specific IMU implementation classes
 * These methods must be implemented in the Passthrough class (a "generic" class used by the code that 
 * chooses the right IMU type based on the parameters in Constants), as well clases specific to the
 * implementation of the IMU type (e.g. IMUNavX and IMUPigeon2)
 * 
 * The IMU object will be created of the type of this interface, and will only contain the methods listed below.
 * That means all desired IMU functionality must be expressed in these methods. If additional functionality
 * is desired, add abstact methods below, and provide their implementation in each of the hardware-specific
 * classes, as well as a Passthrough class, which should simply call an underlying implementation method.
 */
public interface IMUInterface {

    double getPitch();

    double getRoll();

    double getYaw();

    Rotation2d getYawRotation2d();

    double zeroYaw();

    double setYaw(double y);

    double getTurnRate();

    Rotation2d getHeading();
}
