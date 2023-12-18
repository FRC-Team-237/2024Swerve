package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.PassThroughSystems.IMU.IMUInterface;
import frc.robot.PassThroughSystems.IMU.IMUNavX;
import frc.robot.PassThroughSystems.IMU.IMUPigeon2;

/*
 * The IMU object should be instantiated from this class. The class has a selector that will
 * create additional hardware-specific objects. However, other subsystems and commands will
 * use this passthrough class, which exposes "standard" IMU methods that should be implemented
 * by all hardware-specific classes. That way the non-IMU code does not need to know which IMU
 * is actually used.
 * 
 * Since IMU represents a separate hardware component, it's defined as a subsystem
 * (via "extends SubsystemBase"). It means that you can add "periodic" code to it.
 * We do not currently need that. However, when used in a competition, you might find it necessary to do so.
 */

public class IMUSubsystem extends SubsystemBase implements IMUInterface {

  private IMUInterface imu; // We will use downcasting to set this - it will point to methods either in NavX
                            // or Pigeon subsystems

  private double trajectoryAdjustmentIMU; // This is the value we need to adjust the IMU by after Trajectory
                                          // is completed

  /**
   * Creates a new IMUSubsystem.
   * This is an IMU Passthrough class, meaning, it provides a class for the
   * generic IMU instantiation.The "switch" implementation chooser below will
   * instantiate hardware-specific
   * implementation, and will map "imu" variable to that object.
   * That means all hardware-specific implementations must implement the same
   * methods, which is enforced via interface IMUInterface. That interface
   * contains all methods that must be
   * declared as "public" both in this class and in the hardware-specific classes.
   * All other methods
   * (if they're needed) should be private.
   */
  public IMUSubsystem() {

    /*
     * Depending on the IMU type specified in Constants, the variable "imu" will
     * point to the instance
     * of the class specific to the hardware you have, e.g. Pigeon2 or NavX
     * All such implementation classes must have public methods specified by the
     * IMUInterface
     */
    switch (Constants.IMUConstants.imuType) {
      case Pigeon2:
        imu = new IMUPigeon2();
        break;
      case NavX:
        imu = new IMUNavX();
        break;
      default:
    }

    imu.zeroYaw(); // TODO: At the start of game, robot must be pointed towards opposite team's side
                   // (This is our zero value). We do not know which side red/blue is on

  }

  /**
   * Note that all IMU methods that take or return values should do so in SI units.
   */

  public double getPitch() {
    return imu.getPitch();
  }

  public double getRoll() {
    return imu.getRoll();
  }

  public double getYaw() {
    return imu.getYaw();
  }

  public Rotation2d getYawRotation2d() {
    return imu.getYawRotation2d();
  }

  public double zeroYaw() {
    return imu.zeroYaw();
  }

  public double setYaw(double y) {
    return imu.setYaw(y);
  }

  /**
   * This method is used when we want to "snap" the chassis to a trajectory start, meaning
   * assuming that the robot is at the starting point of the trajectory.
   * Here we remember starting Yaw before trajectory so it can be restored
   * back after trajectory
   * @param y - starting Yaw of the trajectory
   * @return - old value of the Yaw (we do not currently use it)
   */
  public double setYawForTrajectory(double y) {
    trajectoryAdjustmentIMU = RobotContainer.imuSubsystem.getYaw() - y;
    return imu.setYaw(y);
  }

  /**
   * Once the trajectory is done, we want to readjust the Yaw considering the value that we "remember", so
   * the field-centric drive axis will not change. That may allow one to drive automated trajectories in teleop
   * without losing the Yaw direction.
   */
  public void restoreYawAfterTrajectory() {
    imu.setYaw(RobotContainer.imuSubsystem.getYaw() + trajectoryAdjustmentIMU);
  }

  public double getTurnRate() {
    return imu.getTurnRate();
  }

  public Rotation2d getHeading() {
    return imu.getHeading();
  }

}
