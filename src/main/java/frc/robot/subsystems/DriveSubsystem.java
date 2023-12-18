// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Constants.SwerveChassis.*;

public class DriveSubsystem extends SubsystemBase {

  public SwerveModule[] swerveMods;

  public SwerveDriveOdometry swerveOdometry;
  public SwerveDrivePoseEstimator swervePoseEstimator;
  
  private Field2d _field = new Field2d(); 

  /**
   * Creates a new DriveSubsystem.
   * In a Swerve this subsystem represents chassis of the Swerve drive.
   * Here we organize the Swerve modules in a proper array object, and define
   * methods that can command
   * individual swerve modules.
   * If a different subsystem needs to operate with the individual swerve module,
   * it needs to go through
   * this subsystem, as it knows which module is responsible for which wheel.
   * If one wants to apply changes to the calculated power and angle for the
   * individual swerve modules,
   * such as implementing a joystick deadzone in order to reduce excessive wheel
   * movements, this
   * should be done here as well. Such decisions should be done by the chassis,
   * not by the individual
   * swerve modules. The swerve modules essentially should simply do what they're
   * told. The only optimization
   * that may be implemented on the swerve module level is angle rotaiton
   * minimization, because it does not
   * change the result of the state change.
   */
  public DriveSubsystem() {

    swerveMods = new SwerveModule[] {
        new SwerveModule(0, SwerveModuleConstants.MOD0), // front left
        new SwerveModule(1, SwerveModuleConstants.MOD1), // front right
        new SwerveModule(2, SwerveModuleConstants.MOD2), // rear left
        new SwerveModule(3, SwerveModuleConstants.MOD3) // rear right
    };

    // When the robot is turned on, both the IMU and drive encoders will be set to
    // 0.
    // So, the initial odometry X,Y,Angle will be set to 0,0,0
    // This may need to be updated later either by th auto-driviing routines, or by
    // camera inputs based on the AprilTags
    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveChassis.SWERVE_KINEMATICS,
        RobotContainer.imuSubsystem.getYawRotation2d(), getPositions());

    // This object will help tracking Swerve pose changes based on the odometry
    // So it will likely be used only for telemetry
    swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveChassis.SWERVE_KINEMATICS,
        RobotContainer.imuSubsystem.getYawRotation2d(), getPositions(), new Pose2d());

  }

  public double telemetryAngleEncoder(int modnumber) {
    return swerveMods[modnumber].telemetryAngleEncoder();
  }

  public double telemetryAngleEncoderSI(int modnumber) {
    return swerveMods[modnumber].telemetryAngleEncoderSI();
  }

  public double telemetryDriveEncoder(int modnumber) {
    return swerveMods[modnumber].telemetryDriveEncoder();
  }

  // Used only for motor testing; run motor forward, 0.3 power
  public void testDriveMotorEncoderPhase(int modnumber) {
    swerveMods[modnumber].testDriveMotorApplyPower(0.3);
  }

  // Used only for motor testing; run motor forward, 0.3 power
  public void testAngleMotorEncoderPhase(int modnumber) {
    swerveMods[modnumber].testAngleMotorApplyPower(0.3);
  }

  public void stopDriveMotor(int modnumber) {
    swerveMods[modnumber].DriveMotorApplyPower(0);
  }

  public void stopAngleMotor(int modnumber) {
    swerveMods[modnumber].AngleMotorApplyPower(0);
  }

  /**
   * We use this method at the end of the trajectory commands.
   * Note that it specifiesthe Omega component. That means it will rotate the
   * wheels
   * to the "zero" position as well, not just cut off the power to the motors.
   * If this behavior is undesirable, change it to the lopp that stops motors
   * instead
   * by caling stopDriveMotor and stopAngleMotor.
   */
  public void stopRobot() {
    drive(0, 0, 0, true);
  }

  /**
   * This method calculates the desired state for each swerve module depending on
   * joystic inputs.
   * In other words, it translates the joystic inputs into the desired state for
   * each of the individual swerve modules.
   * X and Y velocity values need to be submitted from a field point of view,
   * where the 0,0 coordinates are in the
   * left lower corner of the field.
   * 
   * @param xVelocity_m_per_s
   * @param yVelocity_m_per_s
   * @param omega_rad_per_s
   * @param fieldcentric      - if true, use field-centric swerve; otherwise -
   *                          robot-centric
   */
  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s, boolean fieldcentric) {
    SwerveModuleState[] swerveModuleStates;

    if (fieldcentric) { // field-centric swerve
      swerveModuleStates = SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xVelocity_m_per_s,
              yVelocity_m_per_s,
              omega_rad_per_s,
              Rotation2d.fromDegrees(RobotContainer.imuSubsystem.getYaw())));
    } else { // robot-centric swerve; does not use IMU
      swerveModuleStates = SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(
          new ChassisSpeeds(
              xVelocity_m_per_s,
              yVelocity_m_per_s,
              omega_rad_per_s));
    }

    // Because the resulting power is a vector sum of the robot direction and
    // rotation, it's possible that
    // the resulting vector would exceed absolute scalar value of 1. And we need to
    // keep the power in the -1..+1 range.
    // Hence if any of the vectors have a scalar value greater than 1, we need to
    // divide all of them by the largest scalar value
    // of a vector we have. That will preserve the direction of the robot even it is
    // concurrently combioned with the
    // holonomic rotation.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveChassis.MAX_VELOCITY);

    for (SwerveModule mod : swerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()]);
    }

  }
  /***
   * This function translates the desired Relative Chassis Speed into the speeds for the swerve modules. 
   * this only works in ROBOT Relative mode. It will mostly be used by the Path following commands. 
   * @param speed
   */
  public void driveRobotRelative(ChassisSpeeds speed) {
    SwerveModuleState[] swerveModuleStates;
    swerveModuleStates = SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(
        speed
    ); 
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveChassis.MAX_VELOCITY);
    for (SwerveModule mod : swerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()]);
    }
  }

  /**
   * This method sets desired states for the individual swerve modules, specifying
   * angle for the angle motor and power for the drive motor.
   * However, instead of actually driving, it will simply print what did we tell
   * individual swerve modules. So, it should be used for testing.
   * 
   * @param swerveModuleStates - array of the desired swerve module states
   */
  public void setDesiredStatesCalibration(SwerveModuleState[] swerveModuleStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveChassis.MAX_VELOCITY);

    for (SwerveModule mod : swerveMods) {
      mod.setDesiredStateCalibration(swerveModuleStates[mod.getModuleNumber()]);
    }

  }

  /**
   * This method sets desired states for the individual swerve modules, specifying
   * angle for the angle motor and power for the drive motor.
   * In other words, this method tells each individual swerve module what to do
   * next, and is directly involved in driving.
   * 
   * @param swerveModuleStates - array of the desired swerve module states
   */
  public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveChassis.MAX_VELOCITY);

    for (SwerveModule mod : swerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()]);
    }

  }

  /**
   * Creates an array of the swerve module positions (one array element per swerve
   * module)
   * It is only used in odometry calculations, meaning, is only used for
   * automated/trajectory driving
   * and not for teleop/manual driving
   * 
   * @return SwerveModulePosition[] - array of the SwerveModulePosition objects
   *         (WPILIB)
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < positions.length; i++) {
      positions[i] = swerveMods[i].getPosition();
    }
    return positions;
  }

  /**
   * Set odometry to a specified field-centric Pose2d
   * You may need to do so for the trajectory driving, if you want the robot to
   * assume being at the
   * start of the trajectory.
   * Be aware that on-going odometry updates use IMU. So, your odometry yaw may
   * change incorrectly
   * later if the current yaw is not reset properly on the IMU first.
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(RobotContainer.imuSubsystem.getYawRotation2d(), getPositions(), pose);
  }

  /**
   * Field Centric Pose of the chassis
   * We get it from odometry, rather than sensors. That means commands that use it
   * must ensure that
   * odometry was properly updated.
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[swerveMods.length];
    for (int i = 0; i< swerveModuleStates.length; i++)
    {
      swerveModuleStates[i] = swerveMods[i].getState(); 
    }
    return Constants.SwerveChassis.SWERVE_KINEMATICS.toChassisSpeeds(swerveModuleStates); 
  }

  /**
   * Print odometry values from the current state of the odometry object
   * (ServeDriveOdometry).
   * It's used only for telemetry. Use the getPose() method to get the values
   * returned.
   */
  public void odometryTelemetry() {
    RobotContainer.myStringLog.append("Odometry: " + swerveOdometry.getPoseMeters());
  }

  /**
   * This method was designed to print proposed update to the odometry, and can be
   * used for testing
   * if you suspect the odometry values you're submitting are wrong. It may be
   * usefult in
   * catching issues related to the units of measure conversion, or measure
   * imprecission in conversion from
   * the encoder units to the SI units
   * 
   * @param r - rotation of the chassis in Rotation2d
   * @param s - array of positions for each individual swerve module
   */
  public void odometryCommandTelemetry(Rotation2d r, SwerveModulePosition[] s) {
    RobotContainer.myStringLog.append("Odometry Command Rotation: " + r);

    for (int i = 0; i < 4; i++) {
      RobotContainer.myStringLog.append("Odometry Command Module: " + i + " " + s[i]);
    }

  }

  public void resetPoseEstimator(Pose2d pose) {
    swervePoseEstimator.resetPosition(RobotContainer.imuSubsystem.getYawRotation2d(), getPositions(), pose);
  }

  public Pose2d getPoseEstimate() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * This method updates swerve odometry. Note that we do not update it in a
   * periodic method
   * to reduce the CPU usage, since the odometry is only used for trajectory
   * driving.
   * If you use odometry for other purposes, such as using vision to track game
   * elements,
   * either update the odometry in periodic method, or update it from appropriate
   * commands.
   * 
   * The telemetry will print the IMU and swerve positions we send to odometry
   * along with
   * the same information obtained from telemetry after update.
   * It may help troubleshooting potential odometry update issues (e.g. units of
   * measure issues etc)
   * 
   * Please, note that use of telemetry may significantly increase CPU usage, and
   * can ultimately result
   * in a packet loss. We recommend disabling excessive telemetry for the
   * competition.
   */
  public void updateTrajectoryOdometry() {
    if (SwerveTelemetry.odometryTelemetryPrint) {
      RobotContainer.myStringLog.append("PoseSupplier: " + getPose());
      Rotation2d r = RobotContainer.imuSubsystem.getYawRotation2d();
      SwerveModulePosition[] s = getPositions();
      odometryCommandTelemetry(r, s);
      swerveOdometry.update(r, s);
      odometryTelemetry();
    } else {
      swerveOdometry.update(RobotContainer.imuSubsystem.getYawRotation2d(), getPositions());
    }
  }

  /**
   * This method was designed solely for the test comand so you can see whether
   * odometry was updated correctly.
   * It updates gyroAngle to 10 degrees, then reads it and prints right away. If
   * the result is not 10 degrees,
   * something must be wrong.
   */
  public void testOdometryUpdates() {
    swerveOdometry.update(
        Rotation2d.fromDegrees(10),
        getPositions());
    odometryTelemetry();
  }

  public Command followPathCommand(String pathName)
  {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName); 
    return new FollowPathWithEvents(
      new FollowPathHolonomic(
        path,
        this::getPose,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        Constants.SwerveChassis.PATH_FOLLOWER_CONFIG,
        this
        ),
        path,
        this::getPose
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO: We may want to update the robot odometry based the cameras and
    // AprilTags

  }
}
