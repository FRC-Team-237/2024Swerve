// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {

  }

  /**
   * If you turn a wheel counnter-clockwise the angle value and SI value should increase
   */
  public void updateOdometryTelemetry() {
    for (int i =0; i<4; i++){
      SmartDashboard.putNumber("S"+i+" Angle Encoder", RobotContainer.driveSubsystem.telemetryAngleEncoder(i));
      SmartDashboard.putNumber("S"+i+" Angle Encoder SI", RobotContainer.driveSubsystem.telemetryAngleEncoderSI(i));
      SmartDashboard.putNumber("S"+i+" Drive Encoder", RobotContainer.driveSubsystem.telemetryDriveEncoder(i));
      
    }
    
  }

  public void updateIMUTelemetry() {
    SmartDashboard.putNumber("IMU Yaw", RobotContainer.imuSubsystem.getYaw());
  }


  public void updateAllDisplays(){
    updateOdometryTelemetry();
    updateIMUTelemetry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}
