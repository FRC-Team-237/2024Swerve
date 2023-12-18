package frc.robot.PassThroughSystems.Motor;

import com.revrobotics.CANSensor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Constants.PIDConstantsForSwerveModules.NEOAngle;
import frc.robot.Constants.PIDConstantsForSwerveModules.SRXAngle;
import frc.robot.Constants.SwerveChassis.NEOSwerveConfiguration;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstants;

/*
 * This is a specific base motor implementation for the motors connected to SparkMAX NEOs
 */
public class BaseMotorNEO implements BaseMotorInterface {
    private CANSparkMax motorNEO;
    private int CANID;
    private SwerveModuleConstants cAngle;
    private RelativeEncoder driveEncoder;
    private SparkMaxAbsoluteEncoder angleEncoder;
    private SparkMaxPIDController pid;

    public BaseMotorNEO(int CANID) {
        System.out.println("**** Activating SparkMAX NEO CANID:" + CANID);

        motorNEO = new CANSparkMax(CANID, MotorType.kBrushless);

        driveEncoder = motorNEO.getEncoder();
        angleEncoder = motorNEO.getAbsoluteEncoder(Type.kDutyCycle);

        this.CANID=CANID;



    }

    public static double calculateMetersPerRotation(
      double wheelDiameter, double driveGearRatio, double pulsePerRotation)
    {
    return (Math.PI * wheelDiameter) / (driveGearRatio * pulsePerRotation);
    }

    public static double calculateDegreesPerSteeringRotation(
      double angleGearRatio, double pulsePerRotation)
    {
        return 360 / (angleGearRatio * pulsePerRotation);
    }

    public void configureDriveMotor(Constants.SwerveChassis.SwerveModuleConstants c) {


        motorNEO.restoreFactoryDefaults();
        motorNEO.clearFaults();

        motorNEO.setInverted(c.isDriveMotorInverted());
        //driveEncoder.setInverted(c.getDriveMotorSensorPhase());

        pid = motorNEO.getPIDController();

        driveEncoder.setPositionConversionFactor((calculateMetersPerRotation(
            SwerveChassis.WHEEL_DIAMETER,
            SwerveChassis.DRIVE_GEAR_RATIO,
            NEOSwerveConfiguration.DRIVE_PULSE_PER_ROTATION)));
        driveEncoder.setVelocityConversionFactor((calculateMetersPerRotation(
            SwerveChassis.WHEEL_DIAMETER,
            SwerveChassis.DRIVE_GEAR_RATIO,
            NEOSwerveConfiguration.DRIVE_PULSE_PER_ROTATION)) / 60);

        motorNEO.setCANTimeout(0);

        motorNEO.enableVoltageCompensation(NEOSwerveConfiguration.nominalVoltage);
        motorNEO.setSmartCurrentLimit(NEOSwerveConfiguration.driveMotorCurrentLimit);
        motorNEO.setOpenLoopRampRate(NEOSwerveConfiguration.rampRate);
        motorNEO.setClosedLoopRampRate(NEOSwerveConfiguration.rampRate);

        pid.setPositionPIDWrappingEnabled(true);
        pid.setPositionPIDWrappingMinInput(NEOSwerveConfiguration.minInput);
        pid.setPositionPIDWrappingMaxInput(NEOSwerveConfiguration.maxInput);


        pid.setP(NEOAngle.kP);
        pid.setI(NEOAngle.kI);
        pid.setD(NEOAngle.kD);
        pid.setFF(NEOAngle.kF);
        pid.setIZone(NEOAngle.kiz);
        pid.setOutputRange(NEOAngle.outputMin, NEOAngle.outputMax);

        motorNEO.burnFlash();

        motorBrakeMode();
    }

    public void configureAngleMotor(SwerveModuleConstants c) {

        this.cAngle=c;

        motorNEO.restoreFactoryDefaults();
        motorNEO.clearFaults();

        motorNEO.setInverted(c.isAngleMotorInverted());

        pid = motorNEO.getPIDController();

        pid.setFeedbackDevice(angleEncoder);

        angleEncoder.setPositionConversionFactor(NEOSwerveConfiguration.maxInput);
        angleEncoder.setVelocityConversionFactor(NEOSwerveConfiguration.maxInput);

        angleEncoder.setInverted(true);

        pid.setPositionPIDWrappingEnabled(true);
        pid.setPositionPIDWrappingMinInput(NEOSwerveConfiguration.minInput);
        pid.setPositionPIDWrappingMaxInput(NEOSwerveConfiguration.maxInput);

        // Configure PID values

        pid.setP(NEOAngle.kP);
        pid.setI(NEOAngle.kI);
        pid.setD(NEOAngle.kD);
        pid.setFF(NEOAngle.kF);
        pid.setOutputRange(NEOAngle.outputMin, NEOAngle.outputMax);

        motorNEO.setIdleMode(IdleMode.kBrake);
        //motorNEO.set

        motorNEO.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        motorNEO.burnFlash();

        
        motorBrakeMode();

        // if (CANID == 1) {
            //setAngleMotorChassisAngleSI(0);
        // }
        //setAngleMotorChassisAngleSI(0);
    }

    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    public double getAngleEncoderPosition() {
        return angleEncoder.getPosition();
    }

    public double getAngleEncoderPositionCorrected() {
        return angleEncoder.getPosition() + cAngle.getAngleOffset();
    }

    public double getDriveEncoderVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAngleEncoderVelocity() {
        return angleEncoder.getVelocity();
    }

    public double getDriveEncoderPositionSI() {
       
        return getDriveEncoderPosition()*Constants.SwerveChassis.NEOSwerveConfiguration.metersPerTick;
    }

    public double getAngleEncoderPositionSI() {
      
        return getAngleEncoderPosition()*Constants.SwerveChassis.NEOSwerveConfiguration.degreePerTick;
    }

    public double getAngleEncoderPositionSICorrected() {
      
        return (getAngleEncoderPosition() + cAngle.getAngleOffset())*Constants.SwerveChassis.NEOSwerveConfiguration.degreePerTick;
    }

    public double getDriveEncoderVelocitySI() {
   
        return getDriveEncoderVelocity()*Constants.SwerveChassis.NEOSwerveConfiguration.metersPerTick;
    }

    public double getAngleEncoderVelocitySI() {
       
        return getAngleEncoderVelocity()*Constants.SwerveChassis.NEOSwerveConfiguration.degreePerTick;
    }

    public void setAngleMotorChassisAngleSI(double angle) {
        System.out.println("T:"+degreesToTicks(angle) + " A: "+angle);
        motorNEO.getPIDController().setReference(degreesToTicks(angle), ControlType.kPosition);
        
    }

    public void testMotorApplyPower(double power) {
        motorNEO.set(power);
    }

    public void applyPower(double power) {
        motorNEO.set(power);
    }

    private void motorBrakeMode(){
        motorNEO.setIdleMode(IdleMode.kBrake);
    }

    private double degreesToTicks(double degrees) {
        return ((degrees+cAngle.getAngleOffset()) / NEOSwerveConfiguration.degreePerTick)  
         % 
        (NEOSwerveConfiguration.ticksPerFullRotation);
    }

     

    


}
