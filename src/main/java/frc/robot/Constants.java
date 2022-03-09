// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;

import frc.robot.utils.Conversion;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Conversion.inchesToMeters(20.768);

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Conversion.inchesToMeters(20.768);

    /**
     * The free speed of a Falcon 500 Motor
     * 
     * In actual RPM, not ticks
     */
    public static final double FALCON_500_FREE_SPEED = 6380;
    /**
     * The gear ratio of the Swerve Module
     * 
     * From L1-L4
     */
    public static final GearRatio GEAR_RATIO = GearRatio.L2;
    /**
     * The type of Swerve Module used
     * 
     * From L1-L4
     */
    public static final ModuleConfiguration MODULE_CONFIGURATION = SdsModuleConfigurations.MK4I_L2; 

    /**
     * CAN ID of the PigeonIMU
     */
    public static final int DRIVETRAIN_PIGEON_ID = 15;
    

    // FRONT LEFT : Florida
    public static final int FRONT_LEFT_DRIVE_MOTOR = 8;
    public static final int FRONT_LEFT_STEER_MOTOR = 7;
    public static final int FRONT_LEFT_STEER_ENCODER = 12;
    public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(303.92578125); //,318.515  195 // FIXME Measure and set front left steer offset //192.205810546875

    // FRONT RIGHT : France
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_STEER_MOTOR = 1;
    public static final int FRONT_RIGHT_STEER_ENCODER = 9;
    public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(18.104296875000003); //25.4882,97 // FIXME Measure and set back left steer offset //9.3109130859375

    // BACK RIGHT : Railroad
    public static final int BACK_RIGHT_DRIVE_MOTOR = 4;
    public static final int BACK_RIGHT_STEER_MOTOR = 3;
    public static final int BACK_RIGHT_STEER_ENCODER = 10;
    public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(114.778); //41.484, 181 // FIXME Measure and set back right steer offset //305.419921875

    // BACK LEFT : Real Life
    public static final int BACK_LEFT_DRIVE_MOTOR = 6;
    public static final int BACK_LEFT_STEER_MOTOR = 5;
    public static final int BACK_LEFT_STEER_ENCODER = 11;
    public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(97.641796875); //111.086718, 272 // FIXME Measure and set back left steer offset //126.28784179687499
    
    // The Max Velocity for the robot [only in AUTO]
    public static final double T_MAX_VEL = 2.9;

    // The Max Acceleration Value for the robot [only in AUTO]
    public static final double T_MAX_ACCEL = 4;

    public static final double PRECISE_MODE_SCALE = 0.3;

    // The Max Velocity for the robot [only in AUTO]
    public static final double MAX_VEL = 6.5;

    // Climber Constants
    public static final int RIGHT_PIVOT_MOTOR = 20; //CAN SparkMax
    public static final int LEFT_PIVOT_MOTOR = 21; //CAN SparkMax
    public static final int RIGHT_ELEVATOR_MOTOR = 19; //Talon FX
    public static final int LEFT_ELEVATOR_MOTOR = 25; //TalonFX

    private static final double PIVOT_FALCON_GEAR_RATIO = 15/1;
    private static final double ELEVATOR_NEO_GEAR_RATIO = 5/1;
    private static final int FALCON_UNITS_PER_REV = 4096;
    private static final int NEO_UNITS_PER_REV = 42;
    private static final double ELEVATOR_PULLEY_DIAMETER = 1.128;

    public static final double TICKS_PER_PIVOT_DEGREE = (FALCON_UNITS_PER_REV * PIVOT_FALCON_GEAR_RATIO) / 360;
    public static final double TICKS_PER_ELEVATOR_INCH = (NEO_UNITS_PER_REV * ELEVATOR_NEO_GEAR_RATIO) / (ELEVATOR_PULLEY_DIAMETER * Math.PI);
    public static final double MAX_ELEVATOR_HEIGHT = 29.35;

    public static final double PIVOT_KP = 0.15;
    public static final double PIVOT_KI = 0;
    public static final double PIVOT_KD = 0;
    public static final double PIVOT_KF = 0.005;

    public static final double ELEVATOR_KP = 0.1;
    public static final double ELEVATOR_KI = 0;
    public static final double ELEVATOR_KD = 0;
    public static final double ELEVATOR_KF = 0.0;
    public static final double DOWN_ELEVATOR_KP = 4;
    public static final double DOWN_ELEVATOR_KI = 0;
    public static final double DOWN_ELEVATOR_KD = 0;
    public static final double DOWN_ELEVATOR_KF = 0.0075;
    
    private static final double BAR_DIST_X = 24;
    private static final double BAR_DIST_Y = 15.375;
    public static final double BAR_THETA = 90 - Math.toDegrees(Math.atan(BAR_DIST_Y / BAR_DIST_X)); //We may not need this because of a mechanical hardstop but we have it


    // The PID Values for the Limelight driven alignment to the goal.
    public static final double GOAL_ALIGN_KP = 0.008;
    public static final double GOAL_ALIGN_KD = 0.0008;

    // Shooter Constants 
    public static final double tapeHeight = 103.0; // Height of the reflective tape off the ground in INCHES
    public static final double limelightHeight = 27.0; // Height of the limelight off the ground in INCHES
    public static final double limelightAngle = 18.75; // Angle of the limelight in DEGREES
 
    // Superstructure Constants
    // TODO Change if needed
    public static final double kIntakeForwardVoltage = -6.0;
    public static final double kIntakeReverseVoltage = 6.0;
    public static final double kFeederFeedingVoltage = 3.0;
    public static final double kFeederShootingVoltage = 6.0;
    public static final double kFeederReverseVoltage = -6.0;
    public static final double kFeederAdjustVoltage = -0.4;
    // TODO to invert these properly
    public static final double kCenteringWheelForwardVoltage = -12.0;
    public static final double kCenteringWheelReverseVoltage = 6.0;

    // Shooter Velocity Goal Constants
    public static final double kFenderShotVelocity = 2300; //4300
    // public static final double kTarmacShotVelocity = 5000;
    public static final double kLongShotVelocity = 3850; 
    public static final double kWallShotVelocity = 3675;

    public static final double kShooterP = 0.06; //0.1
    public static final double kShooterI = 0;
    public static final double kShooterD = 0;
    public static final double kShooterFF = 0.05;


}
