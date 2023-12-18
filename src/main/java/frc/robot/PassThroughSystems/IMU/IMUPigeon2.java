package frc.robot.PassThroughSystems.IMU;

import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class IMUPigeon2 implements IMUInterface {


    private static WPI_Pigeon2 pigeon2;
    private static Pigeon2_Faults pigeonFaults = new Pigeon2_Faults();
    private double[] xyz = new double[3]; // so not to allocate one every time

    /** Creates a new PidgeonIMUSubsystem. 
     * This is a Pigeon-2 - specific implementation of the IMUPassthroughSubsystem
     * It must implement methods from the IMUInterface
    */
    public IMUPigeon2() {

        System.out.println("**** Activating Pidgeon2 IMU");
        pigeon2 = new WPI_Pigeon2(Constants.IMUConstants.Pigeon2Constants.pigeonIMUId);
    }
      
    public double getPitch() {
        // double[] ypr = new double[3];
        // pigeon2.getYawPitchRoll(ypr);
        // return ypr[1];

        // Front UP - positive Pitch
        return -pigeon2.getPitch();
    }

    /**
     * Gets the roll of the robot (Y axis rotation) (roll is the leaning around the
     * axis that goes straight forward)
     * 
     * @return
     */
    public double getRoll() {
        // double[] ypr = new double[3];
        // pigeon2.getYawPitchRoll(ypr);
        // return ypr[2];

        // Left UP - positive Roll
        return pigeon2.getRoll();
    }

    /**
     * Gets the yaw of the robot (Z axis rotation) (yaw is the direction that the
     * robot is facing around an axis that shoots straight up)
     * 
     * @return
     */
    public double getYaw() {
        //double[] ypr = new double[3];
        //pigeon2.getYawPitchRoll(ypr);
        //System.out.println(ypr[0]);
        //return ypr[0];

        return pigeon2.getYaw(); // With Pigeon2 this method returns values in degrees
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
        double previousYaw = getYaw();
        pigeon2.setYaw(0);
        return previousYaw;
    }

    public double setYaw(double y) {
        double previousYaw = getYaw();
        pigeon2.setYaw(y);
        return previousYaw;
    }

    public Rotation2d getRotation2d() {
        return pigeon2.getRotation2d();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        pigeon2.setYaw(0);
        System.out.println("Yaw and Fused Heading set");
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180 in Rotation2d format
     */
    public Rotation2d getHeading() {
        return pigeon2.getRotation2d();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -pigeon2.getRate();
    }
}
