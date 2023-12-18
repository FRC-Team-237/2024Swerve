package frc.robot.PassThroughSystems.Motor;

import frc.robot.Constants;
import frc.robot.Constants.SwerveChassis.BaseMotorControllerTypes;

/*
 * The motor object should be instantiated from this class. The class has a selector that will
 * create additional hardware-specific objects. However, other subsystems and commands will
 * use this passthrough class, which exposes "standard" motor-related methods that should be implemented
 * by all hardware-specific classes. That way the non-motor-specific code does not need to know which
 * motor type is actually used.
 * 
 * Since we often want encoders directly connected to the motor controllers in order to use
 * hardware PID loops, we expose encoders through the motor object, rather than a separate encoder object.
 * However, if desired, feel free to make the code even more generic by separating encoders into a
 * subsystem.
 * 
 * While technically this class is not really needed (if you move the "switch" motor type selector logic to the
 * class that needs to instantiate motors, such as SwerveModule), this class allows separating individual
 * motor logic from the Swerve logic, albeit at the expense of the additional objects and extra calls.
 * 
 * And extra call occurs because the SwerveModule would call a method of this class when it wants it to
 * do something, and this class' method will call a method in a specific implementation with the same name.
 * E.g. a call to the "configureDriveMotor" of this class will simply result in calling "configureDriveMotor"
 * of specific implementation, such as the same method in BaseMotorTalonSRX.
 */
public class BaseMotorPassthrough implements BaseMotorInterface {
  private BaseMotorInterface baseMotorInterface; // downcasting to the individual motor types

  /** Creates a new IMUSubsystem. */
  public BaseMotorPassthrough(BaseMotorControllerTypes motorType, int CANID) {

    switch (motorType) {
      case TALON_SRX:
        baseMotorInterface = new BaseMotorTalonSRX(CANID);
        break;
      case SPARKMAX:
        baseMotorInterface = new BaseMotorNEO(CANID);
        break;
      default:

    }
  }

  public void configureDriveMotor(Constants.SwerveChassis.SwerveModuleConstants c) {
    baseMotorInterface.configureDriveMotor(c);
  }

  public void configureAngleMotor(Constants.SwerveChassis.SwerveModuleConstants c) {
    baseMotorInterface.configureAngleMotor(c);
  }

  public double getDriveEncoderPosition() {
    return baseMotorInterface.getDriveEncoderPosition();
  }

  public double getAngleEncoderPosition() {
    return baseMotorInterface.getAngleEncoderPosition();
  }

  public double getAngleEncoderPositionCorrected() {
    return baseMotorInterface.getAngleEncoderPosition();
  }

  public double getDriveEncoderVelocity() {
    return baseMotorInterface.getAngleEncoderVelocity();
  }

  public double getAngleEncoderVelocity() {
    return baseMotorInterface.getAngleEncoderVelocity();
  }

  public double getDriveEncoderPositionSI() {
    return baseMotorInterface.getDriveEncoderPositionSI();
  }

  public double getAngleEncoderPositionSI() {
    return baseMotorInterface.getAngleEncoderPositionSI();
  }

  public double getAngleEncoderPositionSICorrected() {
    return baseMotorInterface.getAngleEncoderPositionSI();
  }
  

  public double getDriveEncoderVelocitySI() {
    return baseMotorInterface.getAngleEncoderVelocitySI();
  }

  public double getAngleEncoderVelocitySI() {
    return baseMotorInterface.getAngleEncoderVelocitySI();
  }

  public void setAngleMotorChassisAngleSI(double angle){
    baseMotorInterface.setAngleMotorChassisAngleSI(angle);
  }

  public void testMotorApplyPower(double power) {
    baseMotorInterface.testMotorApplyPower(power);
  }

  public void applyPower(double power) {
    baseMotorInterface.applyPower(power);
  }


}
