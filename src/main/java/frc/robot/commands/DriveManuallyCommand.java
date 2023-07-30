// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Swerve;

public class DriveManuallyCommand extends CommandBase {

  private final DoubleSupplier mVxSupplier;
	private final DoubleSupplier mVySupplier;
	private final DoubleSupplier mOmegaSupplier;

  /** Creates a new DriveManuallyCommand. */
  public DriveManuallyCommand(DoubleSupplier vxSupplier, DoubleSupplier vySupplier, DoubleSupplier omegaSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);

    mVxSupplier = vxSupplier;
		mVySupplier = vySupplier;
		mOmegaSupplier = omegaSupplier;

  }

  private void driveControlTelemetry(double dx, double dy, double dm){
    System.out.print("DX "+ dx);
    System.out.print(" DY "+ dy);
    System.out.println(" Dm "+ dm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xInput = mVxSupplier.getAsDouble();
		double yInput = mVySupplier.getAsDouble();
		double omegaInput = mOmegaSupplier.getAsDouble();

    
    driveControlTelemetry(xInput, yInput, omegaInput);
    
    RobotContainer.driveSubsystem.drive(
      xInput * Swerve.MAX_SPEED,
      yInput * Swerve.MAX_SPEED,
      omegaInput * Swerve.MAX_ANGULAR_VELOCITY
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}