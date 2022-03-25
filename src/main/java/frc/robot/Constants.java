// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.Conversion;
import frc.robot.utils.InterpolationTable;
import frc.robot.utils.PIDGains;

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
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(240.52539062499997); //,318.515  195 // FIXME Measure and set front left steer offset //192.205810546875

    // FRONT RIGHT : France
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(12.875); //25.4882,97 // FIXME Measure and set back left steer offset //9.3109130859375

    // BACK RIGHT : Railroad
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(174+180); //41.484, 181 // FIXME Measure and set back right steer offset //305.419921875

    // BACK LEFT : Real Life
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(77.36796875000002d); //111.086718, 272 // FIXME Measure and set back left steer offset //126.28784179687499
    
    // The Max Velocity for the robot [only in AUTO]
    public static final double T_MAX_VEL = 2.9;

    // The Max Acceleration Value for the robot [only in AUTO]
    public static final double T_MAX_ACCEL = 4;

    public static final double PRECISE_MODE_SCALE = 0.3;

    // The Max Velocity for the robot [only in AUTO]
    public static final double MAX_VEL = 6.5;

    // Climber Constants
    public static final int RIGHT_PIVOT_MOTOR = 20; //TalonFX
    public static final int LEFT_PIVOT_MOTOR = 21; //TalonFX
    public static final int RIGHT_ELEVATOR_MOTOR = 19; //Spark/NEO //TODO FIX THIS GIVE IT A CAN IDEA WHEN IT GOES BACK ON 19
    public static final int LEFT_ELEVATOR_MOTOR = 22; //Spark/NEO 20

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
    public static final double limelightHeight = 40.0; // Height of the limelight off the ground in INCHES
    public static final double limelightAngle = 10.0; // Angle of the limelight in DEGREES
 
    // Superstructure Constants
    // TODO Change if needed
    public static final double kIntakeForwardVoltage = -6.0;
    public static final double kIntakeReverseVoltage = 6.0;
    public static final double kFeederFeedingVoltage = 2.0;
    public static final double kFeederShootingVoltage = 5.0;
    public static final double kFeederReverseVoltage = -6.0;
    public static final double kFeederAdjustVoltage = 0.0;
    // TODO to invert these properly
    public static final double kCenteringWheelForwardVoltage = -12.0;
    public static final double kCenteringWheelReverseVoltage = 6.0;

    // Shooter Velocity Goal Constants
    public static final double kFenderShotVelocity = 700; //4300
    public static final double kFenderLowShotVelocity = 500;
    // public static final double kTarmacShotVelocity = 5000;
    public static final double kLongShotVelocity = 2300; 
    public static final double kWallShotVelocity = 3675;

    public static final double kShooterP = 0.06; //0.1
    public static final double kShooterI = 0;
    public static final double kShooterD = 0;
    public static final double kShooterFF = 0.05;

    public static final int kTimeoutMs = 100;

    // Climber stuff
    public static final class Climber {
        public static final double kClimberVelocityConversion = 600 / 2048d;
        public static final double kClimberTimeoutLong = 5.0;
        public static final int kElevatorPIDSlot = 0;
        public static final int kPivotPIDSlot = 0;
        public static final double kPivotForward = -4200; //-3800
        public static final double kMidPivotHold = -4000;
        public static final double kExtensionMidGoal = -69;
        public static final double kExtensionHighGoal = -94;
        public static final double kExtensionBottom = 0;
        public static final double kExtensionDetach = -20;

        public static final TrapezoidProfile.Constraints kEleavatorTrapConstraints =
            new TrapezoidProfile.Constraints(3000.0 / 60.0, 6000.0 / 60.0);
        public static final ElevatorFeedforward kElevatorFFContraints = 
            new ElevatorFeedforward(0.1, -0.16, 1 / 5880.0); // TODO calculate empirically 
    }

    public static final class Shooter {
        // SVA-PID gains
        private static final double kS_f = 0.51681;
        private static final double kV_f = 0.34371;
        private static final double kA_f = 0.047444;
        private static final double kP_f = 0.15; //115;//0.19508; // 0.21785
        private static final double kI_f = 0.0;
        private static final double kD_f = 0.0;
        public static final SimpleMotorFeedforward kFlywheelFFConstraints = 
            new SimpleMotorFeedforward(kS_f, kV_f, kA_f);
        public static final PIDGains kFlywheelPIDGains = 
            new PIDGains(kP_f, kI_f, kD_f);

        private static final double kS_b = 0.25412;
        private static final double kV_b = 0.41051;
        private static final double kA_b = 0.031141;
        private static final double kP_b = 4.7165E-05; // 0.21785
        private static final double kI_b = 0.0;
        private static final double kD_b = 0.0;
        public static final SimpleMotorFeedforward kBackspinFFConstraints = 
            new SimpleMotorFeedforward(kS_b, kV_b, kA_b);
        public static final PIDGains kBackspinPIDGains = 
            new PIDGains(kP_b, kI_b, kD_b);

        public static final double kBackSpinHighShotSpeed = 8000;
        public static final double kBackSpinLowShotSpeed = 200;

        public static InterpolationTable kHoodAngleLookupTable =
            new InterpolationTable()
            .add(150, 36)
            .add(170, 40)
            .add(200, 45)
            .add(250, 45)
            .add(300, 45)
            .add(370, 48)
            .add(400, 52)
            .add(450, 52);

        public static InterpolationTable kShooterRPMLookupTable = 
            new InterpolationTable()
            .add(150, 1545)
            .add(170, 1570)
            .add(200, 1600)
            .add(250, 1661)
            .add(300, 1714)
            .add(370, 1760)
            .add(400, 1850)
            .add(450, 1923);
        /*
          public static InterpolationTable kHoodAngleLookupTable =
            new InterpolationTable()
            .add(155, 31.3)
            .add(170, 33)
            .add(200, 33)
            .add(250, 35)
            .add(300, 40)
            .add(350, 45)
            .add(400, 50);

        public static InterpolationTable kShooterRPMLookupTable = 
            new InterpolationTable()
            .add(155, 1575)
            .add(170, 1610)
            .add(200, 1668)
            .add(250, 1700)
            .add(300, 1691)
            .add(350, 1714)
            .add(400, 1923); */
    
        }
}
