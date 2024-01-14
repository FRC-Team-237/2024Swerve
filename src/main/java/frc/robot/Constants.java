// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	/**
	 * This class contains configuration constants for the chassis, the individual
	 * swerve modules and the motors
	 * We try to distinguish between them by proper naming.
	 * It's likely we will need to do some additional cleanup here to make them easy
	 * to undrstand
	 */
	public static final class SwerveChassis {

		// TRAJECTORY PARAMETERS 3039
		/* Drive Feedforward */
		public static final double DRIVE_KS = 0.11937 / 12;
		public static final double DRIVE_KV = 2.6335 / 12;
		public static final double DRIVE_KA = 0.46034 / 12;

		/*
		 * Drive train properties
		 * All measurements are in meters
		 */
		public static final double TRACK_WIDTH = 0.64; // left to right
		public static final double WHEEL_BASE = 0.64; // front to back
		public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
		public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
		public static final double ANGLE_GEAR_RATIO = 1.0;
		public static final double DRIVE_GEAR_RATIO = 8.0;

		/**
		 * This class lists locating of each of the swerve modules from the center of
		 * the robot.
		 * It is assumed that there are four swerve modules located at the edges of a
		 * rectangle.
		 * It is also assumed that the center of rotation is at the center of that
		 * rectangle.
		 * 
		 * If your center of rotation is far off the center of that rectangle, which may
		 * happen
		 * due to the very scewed CG or uneven traction, you may want to adjust the
		 * numbers below
		 * based on the center of rotation.
		 * The order of the wheels location definition must match the order of the
		 * swerve modules
		 * defined in the DriveSubsystem for the SwerveModule array.
		 */
		public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
				new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

		/*
		 * Ramp Rates and Current Limits. Assumed to be the same for all drivetrain
		 * motors of the
		 * same type/purpose.
		 */
		public static final double DRIVE_CLOSED_LOOP_RAMP = 0;
		public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;
		public static final int ANGLE_MOTOR_SMART_CURRENT = 25;
		public static final double ANGLE_MOTOR_SECONDARY_LIMIT = 40;
		public static final int DRIVE_MOTOR_SMART_CURRENT = 40;
		public static final double DRIVE_MOTOR_SECONDARY_LIMIT = 60;

		/**
		 * Angle Motor PID. Assumed to be the same for all angle motors
		 * These PID constants are only used for auto trajectory driving, and not
		 * teleop.
		 * Changes to these constants will have a substantial impact on the precision of
		 * your
		 * trajectory if it includes holonomic rotation.
		 * Make sure to test the values and adjust them as needed for your robot.
		 */
		public static final double ANGLE_CHASSIS_KP = 6.25;
		public static final double ANGLE_CHASSIS_KI = 0.4;
		public static final double ANGLE_CHASSIS_KD = 0.7;

		public static final double ANGLE_MOTOR_MIN_OUTPUT = -1;
		public static final double ANGLE_MOTOR_MAX_OUTPUT = 1;

		public static final double ANGLE_MOTOR_PID_TIMEOUT = 30; // milliseconds

		public static final double ANGLE_MOTOR_VELOCITY_CONVERSION = 360.0 / 2048.0; // conversion factor from
																								// tick/100ms to
																								// degree/s

		/**
		 * Drive Motor PID. Assumed to be the same for all drive motors
		 * These PID constants are only used for auto trajectory driving, and not
		 * teleop.
		 * We found that changing them a bit will not have a substantial impact on the
		 * trajectory with PathPlanner
		 * even if a trajectory includes a holonomic component.
		 */
		public static final double DRIVE_CHASSIS_KP = 3.0;
		public static final double DRIVE_CHASSIS_KI = 0.05;
		public static final double DRIVE_CHASSIS_KD = 0.01;

		/**
		 * Maximum linear speed of chassis in meters per second
		 * Note that not determining this number precisely up front will not affect your
		 * teleop driving
		 * as the teleop logic will simply use it as a point of reference.
		 * Changing this number will not require any other changes in the teleop code.
		 */
		public static final double MAX_VELOCITY = 3.0;

		/**
		 * Radians per second.
		 * Swerve chassis assumes that the maximum linear speed during rotation is the
		 * same as the
		 * maxiumum linear speed during forward drive.
		 * That means the maximum angular speed can be calculated by dividing the
		 * maximum linear speed by
		 * the radius of rotation, which can be calculated by halving the distance
		 * between the opposing swerve
		 * modules such as the front left and rear right.
		 */
		public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY
				/ (Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE) / 2);

		// For trajectory driving.
		public static final double MAX_ACCELERATION = 2.0;
        public static final double MAX_ANGULAR_ACCELERATION = 0;

		// define the HolonomicPathFollowerConfig for this drive. 
		public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
			new PIDConstants(DRIVE_CHASSIS_KP ,DRIVE_CHASSIS_KI,DRIVE_CHASSIS_KP),
			new PIDConstants(ANGLE_CHASSIS_KP ,ANGLE_CHASSIS_KI, ANGLE_CHASSIS_KD),
			MAX_VELOCITY,
			0.4,
			new ReplanningConfig()
		); 

		/**
		 * Parameters for BaseMotorTalonSRX class
		 * This class is specific to the motors controlled by TalonSRX controller.
		 * Parameters specified here are primarily used in the motor configuration
		 * methods, as well as
		 * getters that translate encoder outputs from encoder-specific units to the SI
		 * units.
		 * Other motor controller implementations will likely have a different set of
		 * constants.
		 */
		public static final class TalonSRXSwerveConfiguration {

			// We assume that all TalonSRX controlers need the same PID and some other
			// hardware configuration parameters
			public static final int kPIDLoopIdx = 0; // Talon Loop ID
			public static final int configureTimeoutMs = 30; // Hardware Talon Configuration TimeoutMs (probably need to
																// be larger than CAN cycle of 20ms)
																// For Talon configuration command it means - you expect
																// a reply from the controller
																// in that time; otherwise assume the error

			// Customize the following values to your prototype
			public static final double metersPerTick = 1.0 / 1462.25; // TODO: measure this number on the robot
			public static final double degreePerTick = 360.0 / 4096.0; // On our swerve prototype 1 angular rotation of
																		// the wheel = 1 full rotation of the encoder

			// Absolute encoder setup
			public static final boolean kDiscontinuityPresent = true;
			public static final int kBookEnd_0 = 910; /* 80 deg */
			public static final int kBookEnd_1 = 1137; /* 100 deg */
			public static final int clicksSRXPerFullRotation = 4096; // rollover on 999 swerve encoder - we use CTR Mag
																		// encoders for angle with 1:1 ratio

			/**
			 * Current limiters
			 * 
			 * In TalonSRX the limiters limit the input current (not the stator current)
			 * and work in the following way:
			 * If the current demand exceeds peak current for more than a specified
			 * duration,
			 * the current will be limited to the continuous limit until the power demand
			 * drops
			 * below the continuous limit. After that the limiter "resets" and will watch
			 * for peak current again.
			 * For instance in this case if the peak demand on the angle motor exceeds 40amp
			 * for more than 1 second,
			 * the max draw on it will be limited to 25amp until the demand falls below
			 * 25amp.
			 * 
			 * The reason for the limits - you want to make sure you do not trip the
			 * breakers, and Rio will not
			 * go into brownout protection mode. Note that PDP breakers do not immediately
			 * trip when you exceed the
			 * power limit written on them.
			 * Good discussions about brownouts as well as breakers are here:
			 * https://docs.wpilib.org/en/latest/docs/software/roborio-info/roborio-brownouts.html
			 * https://www.chiefdelphi.com/t/power-draw-from-motors/368244/7
			 * https://v5.docs.ctr-electronics.com/en/latest/ch13_MC.html#new-api-in-2020
			 */

			public static final int angleContinuousCurrentLimit = 25; // amperes
			public static final int anglePeakCurrentLimit = 40; // amperes
			public static final int anglePeakCurrentDuration = 1000; // Milliseconds
			public static final boolean angleEnableCurrentLimit = true;

			public static final int driveContinuousCurrentLimit = 35; // amperes
			public static final int drivePeakCurrentLimit = 60; // amperes
			public static final int drivePeakCurrentDuration = 500; // Milliseconds
			public static final boolean driveEnableCurrentLimit = true;



		}

		public static final class NEOSwerveConfiguration {
			public static final double metersPerTick = 1.0 / 1462.25; // TODO: measure this number on the robot
			//public static final double ticksPerFullRotation = 2048.0; 
			//public static final double degreePerTick = 360.0 / ticksPerFullRotation; // BOTH are copied over from TalonSRX
			public static final double ticksPerFullRotation = 2 * Math.PI;
			public static final double degreePerTick = 360/ticksPerFullRotation;

			public static final double nominalVoltage = 12.0;

			public static final int angleMotorCurrentLimit = 20;
			public static final int driveMotorCurrentLimit = 40;

			public static final double rampRate = 0.25;

			public static final double minInput = 0;
			public static final double maxInput = 2*Math.PI;

			public static final double positionConversionFactor = 0;

			public static final double DRIVE_PULSE_PER_ROTATION = 42;

			public static final double ANGLE_PULSE_PER_ROTATION = 0;


			//TODO: find that value pulse per rot

			

		}

		/**
		 * The following constants are used to print swerve telemetry. Please, note that
		 * excessive telemetry
		 * will cause excessive CPU usage, and ultimately will result in a packet loss.
		 * If after enabling telemetry you will see CPU approaching 100% in the RIO log,
		 * such settings will not be
		 * usable for extensive driving or competition. These settings were put in place
		 * so a team can
		 * troubleshoot teleop and trajectory driving.
		 */
		public static final class SwerveTelemetry {

			/**
			 * Collect telemetry using Data Log Manager
			 */
			public static final boolean saveDataUsingDataLogManager = false;
			public static final String logFileName = "/home/lvuser/LittleSwerve";

			public static enum SwerveDriveOrTelemetry {
				DRIVE_ONLY,
				TELEMETRY_ONLY,
				DRIVE_AND_TELEMETRY;
			}

			/**
			 * Specify whether telemetry will be printed and/or the robot will apply power
			 * to the motors
			 */
			public static final SwerveDriveOrTelemetry swerveDriveOrTelemetry = SwerveDriveOrTelemetry.DRIVE_ONLY;

			/**
			 * Print odometry telemetry every 20 milliseconds.
			 */
			public static final boolean odometryTelemetryPrint = false;

		}

		/*
		 * Add controller types for each supported motor controller including simulated
		 * ones
		 * Make sure to modify BaseMotorPassthrough.java and add specific implementation
		 * class
		 * under the "Motor" folder
		 */
		public static enum BaseMotorControllerTypes {
			TALON_SRX,
			SPARKMAX;
		}

		/**
		 * Swerve Module Constants for each module including
		 * driveMotorType - type of the motor controller (e.g. TalonSRX vs NEO vs
		 * simulations)
		 * angleMotorType - type of the motor controller (e.g. TalonSRX vs NEO vs
		 * simulations)
		 * driveMotorID - CAN ID of the drive motors
		 * angleMotorID - CAN ID of the rotation motors
		 * angleOffset - Angle deviation of the absolute encoder when the
		 * respective wheel is pointing forward based on the absolute encoder value
		 * driveMotorInverted - is the drive motor inverted
		 * angleMotorInverted - is the angle motor inverted
		 * driveMotorSensorPhaseInverted - is the drive motor sensor phase inverted
		 * angleMotorSensorPhaseInverted - is the angle motor sensor phase inverted
		 * 
		 * For sensor phase we should use PID rule - when the positive power is applied,
		 * the motor should propell the robot "forward" and the corresponding encoder
		 * value should increase. Also for the angle motors the "positive" direction
		 * is counterclockwise.
		 * 
		 * Only include constants that may differ for each motor.
		 * Items that are the same for each motoro or motor type (e.g. PID constants)
		 * should be defined elsewhere.
		 * 
		 * Since this is an ENUM, need to have getter method for each value.
		 */
		public static enum SwerveModuleConstants {
			MOD0( // Front Left
					BaseMotorControllerTypes.TALON_SRX, // Drive motor type
					BaseMotorControllerTypes.SPARKMAX, // Angle motor type
					17, // driveMotorID
					16, // angleMotorID
					(2.51184335) *360.0/NEOSwerveConfiguration.ticksPerFullRotation, // angleOffset
					false, // Inversion for drive motor
					true, // Inversion for angle motor
					false, // Sensor phase for drive motor
					true // Sensor phase for angle motor
			),
			MOD1( // Front Right
					BaseMotorControllerTypes.TALON_SRX, // Drive motor type
					BaseMotorControllerTypes.SPARKMAX, // Angle motor type
					15, // driveMotorID
					14, // angleMotorID
					(0.162798+Math.PI) *360.0/NEOSwerveConfiguration.ticksPerFullRotation, // angleOffset
					false, // Inversion for drive motor
					true, // Inversion for angle motor
					false, // Sensor phase for drive motor
					true // Sensor phase for angle motor

			),
			MOD2( // Back Left
					BaseMotorControllerTypes.TALON_SRX, // Drive motor type
					BaseMotorControllerTypes.SPARKMAX, // Angle motor type
					11, // driveMotorID
					10, // angleMotorID
					(6.162460-Math.PI) *360.0/NEOSwerveConfiguration.ticksPerFullRotation, // angleOffset
					false, // Inversion for drive motor
					true, // Inversion for angle motor
					false, // Sensor phase for drive motor
					true // Sensor phase for angle motor

			),
			MOD3( // Back Right
					BaseMotorControllerTypes.TALON_SRX, // Drive motor type
					BaseMotorControllerTypes.SPARKMAX, // Angle motor type
					13, // driveMotorID
					12, // angleMotorID
					(5.415143-Math.PI) *360.0/NEOSwerveConfiguration.ticksPerFullRotation, // angleOffset
					false, // Inversion for drive motor
					true, // Inversion for angle motor
					false, // Sensor phase for drive motor
					true // Sensor phase for angle motor

			);

			private BaseMotorControllerTypes driveBaseMotorControllerType;
			private BaseMotorControllerTypes angleBaseMotorControllerType;
			private int driveMotorID;
			private int angleMotorID;
			private double angleOffset;
			private boolean driveMotorInverted;
			private boolean angleMotorInverted;
			private boolean driveMotorSensorPhase;
			private boolean angleMotorSensorPhase;

			SwerveModuleConstants(BaseMotorControllerTypes dm, BaseMotorControllerTypes am, int d, int a, double o,
					boolean di, boolean ai, boolean ds, boolean as) {
				this.driveBaseMotorControllerType = dm;
				this.angleBaseMotorControllerType = am;
				this.driveMotorID = d;
				this.angleMotorID = a;
				this.angleOffset = o;
				this.driveMotorInverted = di;
				this.angleMotorInverted = ai;
				this.driveMotorSensorPhase = ds;
				this.angleMotorSensorPhase = as;
			}

			public BaseMotorControllerTypes getDriveMotorControllerType() {
				return driveBaseMotorControllerType;
			}

			public BaseMotorControllerTypes getAngleMotorControllerType() {
				return angleBaseMotorControllerType;
			}

			public int getDriveMotorID() {
				return driveMotorID;
			}

			public int getAngleMotorID() {
				return angleMotorID;
			}

			public double getAngleOffset() {
				return angleOffset;
			}

			public boolean isDriveMotorInverted() {
				return driveMotorInverted;
			}

			public boolean isAngleMotorInverted() {
				return angleMotorInverted;
			}

			public boolean getDriveMotorSensorPhase() {
				return driveMotorSensorPhase;
			}

			public boolean getAngleMotorSensorPhase() {
				return angleMotorSensorPhase;
			}

		} // End ENUM SwerveModuleConstants

	} // End Swerve

	public static final class IMUConstants {

		public static enum IMUTypes {
			Pigeon2,
			NavX,
			ADIS16470
		}

		public static final IMUTypes imuType = IMUTypes.NavX;

		public static final class Pigeon2Constants {
			public static final int pigeonIMUId = 15; // TODO: Make sure Pigeon ID is correct, this value is a
														// placeholder
		}
	} // End IMUConstants

	/**
	 * These Hardware PID constants are used by the individual swerve modules, and
	 * are used only by turn motors.
	 * We do not currently use Hardware PID for manual or trajectory driving.
	 */
	public static final class PIDConstantsForSwerveModules {

		// Hardware PID-related constants for angle motors controlled by TalonSRX
		public static final class SRXAngle {

			public static final int SLOT_0 = 0;
			public static final double kP = 0.75;
			public static final double kI = 0.005;
			public static final double kD = 0.01;
			public static final double kF = 0;
			public static final double Acceleration = 6750; // raw sensor units per 100 ms per second
			public static final double CruiseVelocity = 6750; // raw sensor units per 100 ms
			public static final int Smoothing = 3; // CurveStrength. 0 to use Trapezoidal Motion Profile. [1,8] for
													// S-Curve (greater value yields greater smoothing).
			public static final double DefaultAcceptableError = 5; // Sensor units
			public static final double Izone = 500;
			public static final double PeakOutput = 0.5; // Closed Loop peak output
			public static final double NeutralDeadband = 0.001;
			public static final int periodMs = 10; // status frame period
			public static final int timeoutMs = 30; // status frame timeout
			public static final int closedLoopPeriod = 1; // 1ms for TalonSRX and locally connected encoder

		}

		
		public static final class NEOAngle {

			public static final double kP = 0.4;
			public static final double kI = 0.0;
			public static final double kD = 0.8;
			public static final double kF = 0.0;
			public static final double kiz = 0; // I-zone
			public static final double Acceleration = 6750; // raw sensor units per 100 ms per second
			public static final double CruiseVelocity = 6750; // raw sensor units per 100 ms
			public static final double DefaultAcceptableError = 5; // Sensor units
			public static final double Izone = 250;
			public static final double PeakOutput = 0.5; // Closed Loop peak output
			public static final double NeutralDeadband = 0.001;

			public static final double outputMin= -1.0;
			public static final double outputMax = 1.0;

		}

	}

	/**
	 * Controller-related constants.
	 * Here we define port numbers, axis, deadbands, button numbers and various
	 * ability flags, such as use of the cube driving
	 */
	public static final class OIConstants {
		public static final int driverControllerPort = 0;

		public static final int bblPort = 4;
		public static final int bbrPort = 3;

		public static final int driverInterfaceSwitchButton = 1;

		public static final int robotCentricButton = 2;

		public static ControllerDeviceType driverInterfaceType = ControllerDeviceType.LOGITECH;

		public static enum ControllerDeviceType {
			LOGITECH,
			PS5,
			XBOX
		}

		public static enum ControllerDevice {
			DRIVESTICK(
					0, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			TURNSTICK( // Controls the rotation of the swervebot
					1, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			XBOX_CONTROLLER(
					5, // Port Number for Xbox controller
					ControllerDeviceType.XBOX,
					0.1, // deadband X for Xbox
					0.1, // deadband Y for Xbox       //TODO: ALL DEADBAND FOR XBOX IS PLACEHOLDER
					0.1, // deadband Omega for Xbox
					false, // No cube controller configuration for Xbox yet
					false);

			private ControllerDeviceType controllerDeviceType;
			private int portNumber;
			private double deadbandX;
			private double deadbandY;
			private double deadbandOmega;
			private boolean cubeControllerLeftStick;
			private boolean cubeControllerRightStick;

			ControllerDevice(
					int pn,
					ControllerDeviceType cdt,
					double dx,
					double dy,
					double dm,
					boolean ccL,
					boolean ccR) {
				this.portNumber = pn;
				this.controllerDeviceType = cdt;
				this.deadbandX = dx;
				this.deadbandY = dy;
				this.deadbandOmega = dm;
				this.cubeControllerLeftStick = ccL;
				this.cubeControllerRightStick = ccR;
			}

			public ControllerDeviceType getControllerDeviceType() {
				return controllerDeviceType;
			}

			public int getPortNumber() {
				return portNumber;
			}

			public double getDeadbandX() {
				return deadbandX;
			}

			public double getDeadbandY() {
				return deadbandY;
			}

			public double getDeadbandOmega() {
				return deadbandOmega;
			}

			public boolean isCubeControllerLeftStick() {
				return cubeControllerLeftStick;
			}

			public boolean isCubeControllerRightStick() {
				return cubeControllerRightStick;
			}
		}
	}

}
