package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveChassis;
import frc.robot.subsystems.DriveSubsystem;


public class TurnToAngleZeroHeadingCommand extends CommandBase {
	private final double kP = 0.5;
	private final double kI = 0;
	private final double kD = 0;
	Rotation2d angle = Rotation2d.fromDegrees(0);
	//private double kMaxSpeed = Constants.SwerveChassis.MAX_ANGULAR_VELOCITY;
	//private double kMaxAccel = Constants.SwerveChassis.MAX_ACCELERATION;
	private double kMaxSpeed = 360;
	private double kMaxAccel = 720;
	private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAccel);
	private ProfiledPIDController profiledPID = new ProfiledPIDController(kP, kI, kD, constraints);
	private boolean seeCone = false;

	public TurnToAngleZeroHeadingCommand(Rotation2d angle) {
		this.angle = angle;
		addRequirements(RobotContainer.driveSubsystem);
		profiledPID.enableContinuousInput(0, 360);
	}

	@Override
	public void initialize() {
		seeCone = RobotContainer.networkTablesSubsystem.seeCone();
		profiledPID.reset(RobotContainer.imuSubsystem.getYaw());
		
	}

	@Override
	public void execute() {
		double omegaDegPerSec = profiledPID.calculate(RobotContainer.imuSubsystem.getYaw());
		if (seeCone) {
			RobotContainer.driveSubsystem.drive(0, 0, Units.degreesToRadians(omegaDegPerSec)* SwerveChassis.MAX_ANGULAR_VELOCITY, true);
		}
		profiledPID.setGoal(RobotContainer.imuSubsystem.getYaw() + RobotContainer.networkTablesSubsystem.getVisionTargetX());
		SmartDashboard.putNumber("***Rotation Command Angle: ", Units.degreesToRadians(omegaDegPerSec)* SwerveChassis.MAX_ANGULAR_VELOCITY);
		System.out.println("***See: "+ seeCone + " ***A: "+ Units.degreesToRadians(omegaDegPerSec)* SwerveChassis.MAX_ANGULAR_VELOCITY);
	}

}
