package frc.robot.PassThroughSystems.IMU;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class IMUNavX implements IMUInterface {

    private AHRS navX;

    /** Creates a new NavXSubsystem. 
     * This is a NavX-specific implementation of the IMUPassthroughSubsystem
     * It must implement methods from the IMUInterface
    */
    public IMUNavX() {

        System.out.println("**** Activating NavX IMU");

        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            // TODO: We need to set up a drivers system output system: perhaps Robot.java
            // should catch this exception
            // Or we could output to the driver system directly, as they do in their example
            // DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(),
            // true);
        }
    }

    /**
     * Gets the pitch of the robot (X axis rotation) (pitch is rotation around the
     * horizontal axis perpendicular to straight forward)
     * 
     * @return The pitch of the robot
     */
    public double getPitch() {
        return navX.getPitch();
    }

    /**
     * Gets the roll of the robot (Y axis rotation) (roll is the leaning around the
     * axis that goes straight forward)
     * 
     * @return
     */
    public double getRoll() {
        return navX.getRoll();
    }

    /**
     * Gets the yaw of the robot (Z axis rotation) (yaw is the direction that the
     * robot is facing around an axis that shoots straight up)
     * 
     * @return
     */
    public double getYaw() {
        return navX.getYaw();
    }

    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    /**
     * Zeroes the yaw of the robot
     * 
     * @return The previous yaw
     */
    public double zeroYaw() {
        double previousYaw = navX.getYaw();
        navX.zeroYaw();
        return previousYaw;
    }

    public double setYaw(double y) {
        double previousYaw = navX.getYaw();
        navX.zeroYaw();
        navX.setAngleAdjustment(y);
        return previousYaw;
    }

    /**
     * Provide heading in degrees with the angle increasing clockwise hence the
     * negative value of getAngle
     * 
     * @return
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-navX.getAngle());
    }

    public AHRS getNavX() {
        return navX;
    }

    public double getTurnRate() {
        return navX.getRate();
    }
}
