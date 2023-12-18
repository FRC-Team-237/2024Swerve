package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;


public class TurnToAngleZeroHeadingCommand extends Command {
	private final double kP = 2;
	private final double kI = 0;
	private final double kD = 0;
	Rotation2d angle = Rotation2d.fromDegrees(0);
	DriveSubsystem driveSubsystem = new DriveSubsystem();
	private double kMaxSpeed = Constants.SwerveChassis.MAX_ANGULAR_VELOCITY;
	private double kMaxAccel = Constants.SwerveChassis.MAX_ACCELERATION;
	private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAccel);
	private ProfiledPIDController profiledPID = new ProfiledPIDController(kP, kI, kD, constraints);

	public TurnToAngleZeroHeadingCommand(Rotation2d angle) {
		this.angle = angle;
		addRequirements(driveSubsystem);
		profiledPID.enableContinuousInput(0, 360);
	}

	@Override
	public void initialize() {
		profiledPID.reset(RobotContainer.imuSubsystem.getYaw());
		profiledPID.setGoal(angle.getDegrees());
	}

	@Override
	public void execute() {
		double omegaDegPerSec = profiledPID.calculate(RobotContainer.imuSubsystem.getYaw());
		driveSubsystem.drive(0, 0, Units.degreesToRadians(omegaDegPerSec), true);
	}

}
