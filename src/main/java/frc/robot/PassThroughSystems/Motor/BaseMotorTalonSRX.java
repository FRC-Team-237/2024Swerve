package frc.robot.PassThroughSystems.Motor;

import java.lang.invoke.ConstantCallSite;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Constants.PIDConstantsForSwerveModules.SRXAngle;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstants;
import frc.robot.Constants.SwerveChassis.TalonSRXSwerveConfiguration;

/*
 * This is a specific base motor implementation for the motors connected to TalonSRX
 */
public class BaseMotorTalonSRX implements BaseMotorInterface {
    private WPI_TalonSRX motorTalonSRX;

    public BaseMotorTalonSRX(int CANID) {

        System.out.println("**** Activating TalonSRX CANID:" + CANID);

        motorTalonSRX = new WPI_TalonSRX(CANID);

    }

    public void configureDriveMotor(Constants.SwerveChassis.SwerveModuleConstants c) {

        motorTalonSRX.configFactoryDefault();
        motorTalonSRX.setInverted(c.isDriveMotorInverted());

        // Encoder configuration
        motorTalonSRX.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,
                Constants.SwerveChassis.TalonSRXSwerveConfiguration.kPIDLoopIdx,
                Constants.SwerveChassis.TalonSRXSwerveConfiguration.configureTimeoutMs);

        motorTalonSRX.setSensorPhase(c.getDriveMotorSensorPhase());

        configureCurrentLimiterDrive();

        motorBrakeMode();

    }

    public void configureAngleMotor(SwerveModuleConstants c) {

        motorTalonSRX.configFactoryDefault();
        motorTalonSRX.setInverted(c.isAngleMotorInverted());

        /** Configure encoder
         * We use CTR Mag encoders directly connected to the TalonSRX.
         * These encoders can be used as both absolute and relative encoders at the same time.
         */
        motorTalonSRX.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute,
                Constants.SwerveChassis.TalonSRXSwerveConfiguration.kPIDLoopIdx,
                Constants.SwerveChassis.TalonSRXSwerveConfiguration.configureTimeoutMs);

        motorTalonSRX.setSensorPhase(c.getAngleMotorSensorPhase());

        initQuadrature();

        configureMotionMagicAngle(c);

        setEncoderforWheelCalibration(c);

        configureCurrentLimiterAngle();

        motorBrakeMode();

        //setAngleMotorChassisAngle(0); //Initialization turn wheels to 0 degrees
        
    }

    public double getDriveEncoderPosition() {
        return motorTalonSRX.getSelectedSensorPosition();
    }

    public double getAngleEncoderPosition() {
        return motorTalonSRX.getSelectedSensorPosition();
    }

    public double getAngleEncoderPositionCorrected() {
        return getAngleEncoderPosition();
    }

    public double getDriveEncoderVelocity() {
        return motorTalonSRX.getSelectedSensorVelocity();
    }

    public double getAngleEncoderVelocity() {
        return motorTalonSRX.getSelectedSensorVelocity();
    }

    public double getDriveEncoderPositionSI() {
        return motorTalonSRX.getSelectedSensorPosition()*Constants.SwerveChassis.TalonSRXSwerveConfiguration.metersPerTick;
    }

    public double getAngleEncoderPositionSI() {
        return motorTalonSRX.getSelectedSensorPosition()*Constants.SwerveChassis.TalonSRXSwerveConfiguration.degreePerTick;
    }

    public double getAngleEncoderPositionSICorrected() {
        return getAngleEncoderPositionSI();
    }

    public double getDriveEncoderVelocitySI() {
        return motorTalonSRX.getSelectedSensorVelocity()*10.0*Constants.SwerveChassis.TalonSRXSwerveConfiguration.metersPerTick;
    }

    public double getAngleEncoderVelocitySI() {
        return motorTalonSRX.getSelectedSensorVelocity()*10.0*Constants.SwerveChassis.TalonSRXSwerveConfiguration.degreePerTick;
    }

    private int getDriveAbsEncoder() {
        return (int) motorTalonSRX.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    }

    public void setAngleMotorChassisAngleSI(double angle){
        motorTalonSRX.set(TalonSRXControlMode.MotionMagic, degreesToTicks(angle));
    }

    public void testMotorApplyPower(double power) {
        motorTalonSRX.set(TalonSRXControlMode.PercentOutput, power);
    }

    public void applyPower(double power) {
        motorTalonSRX.set(TalonSRXControlMode.PercentOutput, power);
    }

     private double degreesToTicks(double degrees) {
        return degrees / TalonSRXSwerveConfiguration.degreePerTick;
    }

    

    private void configureMotionMagicAngle(Constants.SwerveChassis.SwerveModuleConstants c) {

        // Disable motor safety so we can use hardware PID
        motorTalonSRX.setSafetyEnabled(false);

        motorTalonSRX.configNeutralDeadband(SRXAngle.NeutralDeadband, 30);

        motorTalonSRX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, SRXAngle.periodMs,  SRXAngle.timeoutMs);
        motorTalonSRX. setStatusFramePeriod(StatusFrame.Status_10_Targets, SRXAngle.periodMs,  SRXAngle.timeoutMs);

        motorTalonSRX.configPeakOutputForward(+1.0, SRXAngle.timeoutMs);
        motorTalonSRX.configPeakOutputReverse(-1.0, SRXAngle.timeoutMs);
        motorTalonSRX.configNominalOutputForward(0, SRXAngle.timeoutMs);
        motorTalonSRX.configNominalOutputReverse(0, SRXAngle.timeoutMs);

        /* FPID Gains */
        motorTalonSRX.selectProfileSlot(SRXAngle.SLOT_0, 0);
        motorTalonSRX.config_kP(SRXAngle.SLOT_0, SRXAngle.kP, SRXAngle.timeoutMs);
        motorTalonSRX.config_kI(SRXAngle.SLOT_0, SRXAngle.kI, SRXAngle.timeoutMs);
        motorTalonSRX.config_kD(SRXAngle.SLOT_0, SRXAngle.kD, SRXAngle.timeoutMs);
        motorTalonSRX.config_kF(SRXAngle.SLOT_0, SRXAngle.kF, SRXAngle.timeoutMs);

        motorTalonSRX.config_IntegralZone(SRXAngle.SLOT_0, SRXAngle.Izone,  SRXAngle.timeoutMs);
        motorTalonSRX.configClosedLoopPeakOutput(SRXAngle.SLOT_0, SRXAngle.PeakOutput, SRXAngle.timeoutMs);
        motorTalonSRX.configAllowableClosedloopError(SRXAngle.SLOT_0, SRXAngle.DefaultAcceptableError, SRXAngle.timeoutMs);
      
        motorTalonSRX.configClosedLoopPeriod(SRXAngle.SLOT_0, SRXAngle.closedLoopPeriod, SRXAngle.timeoutMs);

        motorTalonSRX.configMotionAcceleration(SRXAngle.Acceleration,SRXAngle.timeoutMs);
        motorTalonSRX.configMotionCruiseVelocity(SRXAngle.CruiseVelocity,SRXAngle.timeoutMs);
        motorTalonSRX.configMotionSCurveStrength(SRXAngle.Smoothing);
    }

    // Current limiter configuration for the angle motor
    private void configureCurrentLimiterAngle() {
        motorTalonSRX.configPeakCurrentLimit(TalonSRXSwerveConfiguration.anglePeakCurrentLimit, TalonSRXSwerveConfiguration.configureTimeoutMs);
		motorTalonSRX.configPeakCurrentDuration(TalonSRXSwerveConfiguration.anglePeakCurrentDuration, TalonSRXSwerveConfiguration.configureTimeoutMs);
		motorTalonSRX.configContinuousCurrentLimit(TalonSRXSwerveConfiguration.angleContinuousCurrentLimit, TalonSRXSwerveConfiguration.configureTimeoutMs);
		motorTalonSRX.enableCurrentLimit(TalonSRXSwerveConfiguration.angleEnableCurrentLimit); // Honor initial setting

    }

    // Current limiter configuration for the drive motor
    private void configureCurrentLimiterDrive() {
        motorTalonSRX.configPeakCurrentLimit(TalonSRXSwerveConfiguration.drivePeakCurrentLimit, TalonSRXSwerveConfiguration.configureTimeoutMs);
		motorTalonSRX.configPeakCurrentDuration(TalonSRXSwerveConfiguration.drivePeakCurrentDuration, TalonSRXSwerveConfiguration.configureTimeoutMs);
		motorTalonSRX.configContinuousCurrentLimit(TalonSRXSwerveConfiguration.driveContinuousCurrentLimit, TalonSRXSwerveConfiguration.configureTimeoutMs);
		motorTalonSRX.enableCurrentLimit(TalonSRXSwerveConfiguration.driveEnableCurrentLimit); // Honor initial setting

    }

    public void initQuadrature() { // Set absolute encoders
        int pulseWidth = motorTalonSRX.getSensorCollection().getPulseWidthPosition();

        if (TalonSRXSwerveConfiguration.kDiscontinuityPresent) {

            /* Calculate the center */
            int newCenter;
            newCenter = (TalonSRXSwerveConfiguration.kBookEnd_0 + TalonSRXSwerveConfiguration.kBookEnd_1) / 2;
            newCenter &= 0xFFF;

            /**
             * Apply the offset so the discontinuity is in the unused portion of
             * the sensor
             */
            pulseWidth -= newCenter;
        }
    }

    /**
     * The CTR Mag encoders we use to track wheel angle can be used in both absolute and relative modes
     * at the same time. The Hardware PID on the TalonSRX, however, is easier to use with relative encoders.
     * So, we read absolute encoders at the start, and set relative encoders so their 0 corresponds to the
     * wheels being lined up and facing forward (0 degree from the forward robot direction).
     * We have not found significant drift/discrepancy between absolute and relative encoder increments, so
     * we do not currently recalibrate relative encoders again during the game.
     * Note that the wheels do not need to be set "forward" at the beginning of the game. The absolute encoder
     * will set the right angle-related value to the relative encoder, since absolute encoders are not set to 0 after
     * power cycle. The drive routines will then change the wheel positions as needed.
    */
    public void setEncoderforWheelCalibration(SwerveModuleConstants c) {
        double difference = getDriveAbsEncoder() - c.getAngleOffset()*4096.0/360.0;
        double encoderSetting = 0.0;

        if (difference < 0) {
            difference += TalonSRXSwerveConfiguration.clicksSRXPerFullRotation;
        }

        if (difference <= TalonSRXSwerveConfiguration.clicksSRXPerFullRotation / 2) {
            encoderSetting = difference;

        } else {
            encoderSetting = difference - TalonSRXSwerveConfiguration.clicksSRXPerFullRotation;
        }

        motorTalonSRX.setSelectedSensorPosition(encoderSetting);

        System.out.println("Set encoder for motor " + c.getAngleMotorID() + " to " + encoderSetting);

    }

    private void motorBrakeMode() {
        motorTalonSRX.setNeutralMode(NeutralMode.Brake);
    }

}
