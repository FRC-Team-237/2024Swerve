package frc.robot.PassThroughSystems.Motor;

import frc.robot.Constants;

/**
 * This interface lists all methods that must be implemented by specific motor controller implementation classes
 * These methods must be implemented in the Passthrough class (a "generic" class used by the code that 
 * chooses the right motor controller type based on the parameters in Constants), as well clases specific to the
 * implementation of the motor controller type (e.g. TalonSRX vs SPARK_MAX).
 * You should be able to use simulation classes here as well, as long as you expose their functionality via mehtods
 * listed in this interface.
 * 
 * The motor controler object will be created of the type of this interface, and will only contain the methods listed below.
 * That means all desired motor controller functionality must be expressed in these methods. If additional functionality
 * is desired, add abstact methods below, and provide their implementation in each of the hardware-specific
 * classes, as well as a Passthrough class, which should simply call an underlying implementation method.
 */
public interface BaseMotorInterface {

    void configureDriveMotor(Constants.SwerveChassis.SwerveModuleConstants c);

    void configureAngleMotor(Constants.SwerveChassis.SwerveModuleConstants c);

    /*
     * The methods below expose encoder values for position and speed.
     * The non-SI methods provide values in raw encoder units 
     * (e.g. in CTR Mag Encoder velocity is measured in ticks per 100ms)
     * The SI methods provide value sin SI units - m/s and degree/s (we may consider modifying the latter to radians/s)
     */
    
    double getDriveEncoderPosition();

    double getAngleEncoderPosition();

    double getDriveEncoderVelocity();

    double getAngleEncoderVelocity();

    double getDriveEncoderPositionSI();

    double getAngleEncoderPositionSI();

    double getDriveEncoderVelocitySI();

    double getAngleEncoderVelocitySI();

    void setAngleMotorChassisAngleSI(double angle); // Moves angle motor to desired angle using PID

    void testMotorApplyPower(double power); // Test method to check inversion and encoders

    void applyPower(double power); // Apply power to the motor. Should be between -1 and 1

}
