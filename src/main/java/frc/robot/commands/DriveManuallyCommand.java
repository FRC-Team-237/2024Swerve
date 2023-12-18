// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveChassis;

public class DriveManuallyCommand extends Command {

  private final DoubleSupplier mVxSupplier;
	private final DoubleSupplier mVySupplier;
	private final DoubleSupplier mOmegaSupplier;
  private final BooleanSupplier mFieldCentrSupplier;

  /** Creates a new DriveManuallyCommand.
   * This is the command used for teleop manual driving
   */
  public DriveManuallyCommand(DoubleSupplier vxSupplier, DoubleSupplier vySupplier, DoubleSupplier omegaSupplier, BooleanSupplier fieldCentrSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);

    mVxSupplier = vxSupplier;
		mVySupplier = vySupplier;
		mOmegaSupplier = omegaSupplier;
    mFieldCentrSupplier = fieldCentrSupplier;

  }

  /**
   * This method man be used when troubleshooting controller inputs
   * @param dx
   * @param dy
   * @param dm
   */
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

    
    //driveControlTelemetry(xInput, yInput, omegaInput);
    
    RobotContainer.driveSubsystem.drive(
      xInput * SwerveChassis.MAX_VELOCITY,
      yInput * SwerveChassis.MAX_VELOCITY,
      omegaInput * SwerveChassis.MAX_ANGULAR_VELOCITY,
      mFieldCentrSupplier.getAsBoolean()
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("DCE:"+interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
