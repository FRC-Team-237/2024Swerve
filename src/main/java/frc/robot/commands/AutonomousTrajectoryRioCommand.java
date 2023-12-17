// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveChassis;

/**
 * Runs trajectory. The command will not update initial odometry of the robot.
 * That should be done by a separate command preceding this one.
 */
public class AutonomousTrajectoryRioCommand extends PPSwerveControllerCommand {
  /** Creates a new AutonomousTrajectoryRioCommand.
   * This command runs the trajectory using PathPlanner holonomic trajectory controller.
   * The trajectory provided to this command must be in a form of PathPlannerTrajectory object.
   * That means that any adjustments, such as reducing its speed, reversal etc must be
   * applied before the trajectory is passed to this command.
   * Note that PathPlanner ends trajectories on time rather that completion of the distance.
   * Therefore, PID constants provided to the PID controller may have an impact on the end result,
   * especially for the holonomic component.
   */

  TrajectoryConfig config;

  PathPlannerTrajectory trajectoryPath;
  
  public AutonomousTrajectoryRioCommand(PathPlannerTrajectory trajectoryPath) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    // Since we extend PPSwerveControllerCommand, we need to call its constructor properly
    // This command was done to provide better control and telemetry over the execution of
    // the PathPlanner trajectory
    super(
      trajectoryPath,
      RobotContainer.driveSubsystem::getPose,
      Constants.SwerveChassis.SWERVE_KINEMATICS,
      new PIDController(Constants.SwerveChassis.DRIVE_CHASSIS_KP,
                        Constants.SwerveChassis.DRIVE_CHASSIS_KI,
                        Constants.SwerveChassis.DRIVE_CHASSIS_KD),
      new PIDController(Constants.SwerveChassis.DRIVE_CHASSIS_KP,
                        Constants.SwerveChassis.DRIVE_CHASSIS_KI,
                        Constants.SwerveChassis.DRIVE_CHASSIS_KD),
      new PIDController(Constants.SwerveChassis.ANGLE_CHASSIS_KP,
                        Constants.SwerveChassis.ANGLE_CHASSIS_KI,
                        Constants.SwerveChassis.ANGLE_CHASSIS_KD),
      RobotContainer.driveSubsystem::setDesiredStates,
      false,
      RobotContainer.driveSubsystem
    );
    this.trajectoryPath = trajectoryPath;
  }

  // Run trajectory with known maximum velocity and acceleration
  /**
   * @param trajectoryName Filename containing trajectory without .path
   * @param maxVelocity    Maximum velocity m/s
   * @param maxAcceleration  Maximum acceleration m/s^2
   */
  public AutonomousTrajectoryRioCommand(String trajectoryName, double maxVelocity, double maxAcceleration){
    this(PathPlanner.loadPath(trajectoryName, new PathConstraints(maxVelocity, maxAcceleration)));
    System.out.println("initalized trajectory: "+ trajectoryName + "V:"+maxVelocity+" A:"+maxAcceleration);
  }

  // Run trajectory with default maximum velocity and acceleration
  /**
   * @param trajectoryName Filename containing trajectory without .path
   */
  public AutonomousTrajectoryRioCommand(String trajectoryName){
    this(PathPlanner.loadPath(trajectoryName, 
      new PathConstraints(SwerveChassis.MAX_VELOCITY, SwerveChassis.MAX_ACCELERATION)));
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    System.out.println("Auto trajectory initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update robot odometry

    //System.out.println("O");

    /**
     * In this example project we only update field odometry when needed, meaning when running
     * automated trajectories. However, you may need to update it in other situations, especially
     * when using vision to determine robot's position on the field.
     */
    RobotContainer.driveSubsystem.updateTrajectoryOdometry();

    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println("*** End trajectory command. Interrupted:"+interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return super.isFinished();

  }
}
