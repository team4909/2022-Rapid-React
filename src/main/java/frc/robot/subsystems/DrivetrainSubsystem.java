// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.*;

import java.util.Map;

public class DrivetrainSubsystem extends SubsystemBase {

    
    private static DrivetrainSubsystem instance = null;

    /**
     * The scale factor that the speed during Auto trajectories will be affected by.
     * <p>
     * This can be increased to go faster in auto, decreased to go slower.
     * Make sure you change in both the PathPlanner gui and in the swerve controller command!
     */
    public static final double AUTO_DRIVE_SCALE = 1;
    /**+
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     * Calculate by: Motor fre speed RPM / 60 * Drive Reduction * Wheel Diameter Meters * pi
     */
    public static final double MAX_VOLTAGE = 12; //Constants.FALCON_500_FREE_SPEED / 60.0 / MODULE_CONFIGURATION.getDriveReduction() * MODULE_CONFIGURATION.getWheelDiameter() * Math.PI;
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = Constants.FALCON_500_FREE_SPEED / 60.0 *
            MODULE_CONFIGURATION.getDriveReduction() *
            MODULE_CONFIGURATION.getWheelDiameter() * Math.PI;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    /**
     * The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
     * cause the angle reading to increase until it wraps back over to zero.
     */
    private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);

    // These are our modules. We initialize them in the initializeMotors method.
    private SwerveModule m_frontLeftModule;
    private SwerveModule m_frontRightModule;
    private SwerveModule m_backLeftModule;
    private SwerveModule m_backRightModule;

    // Absolute Cancoders (1 per module)
    private CANCoder m_frontLeftCanCoder;
    private CANCoder m_frontRightCanCoder;
    private CANCoder m_backLeftCanCoder;
    private CANCoder m_backRightCanCoder;
    
    // Odometry for storing the position of the robot
    private SwerveDriveOdometry m_odometry;

    private ShuffleboardTab m_tab;

    // ChassisSpeeds object to supply the drivetrain with (X, Y, Rotation)
    private ChassisSpeeds m_chassisSpeeds;

    private DrivetrainSubsystem() {
        m_tab = Shuffleboard.getTab("Drivetrain");

        m_chassisSpeeds  = new ChassisSpeeds(0.0, 0.0, 0.0);
        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());

        initilizeEncoders();
        initializeMotors();       
    }

    public void initilizeEncoders(){
        m_frontLeftCanCoder = new CANCoder(Constants.FRONT_LEFT_STEER_ENCODER);
        m_frontRightCanCoder = new CANCoder(Constants.FRONT_RIGHT_STEER_ENCODER);
        m_backLeftCanCoder = new CANCoder(Constants.BACK_LEFT_STEER_ENCODER);
        m_backRightCanCoder = new CANCoder(Constants.BACK_RIGHT_STEER_ENCODER);

        m_frontLeftCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_frontRightCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_backLeftCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_backRightCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        m_frontLeftCanCoder.configSensorDirection(true);
        m_frontRightCanCoder.configSensorDirection(true);
        m_backLeftCanCoder.configSensorDirection(true);
        m_backRightCanCoder.configSensorDirection(true);
    }

    public void initializeMotors(){
        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // Allows you to see the current state of the module on the dashboard.
            m_tab.getLayout("Front Left Module", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 0))
                    .withSize(1, 4)
                    .withPosition(0, 0),
            // L1 - L4 Change in Constants
            GEAR_RATIO,
            // This is the ID of the drive motor
            FRONT_LEFT_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_STEER_OFFSET
        );

        m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            m_tab.getLayout("Front Right Module", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 0))
                    .withSize(1, 4)
                    .withPosition(2, 0),
            GEAR_RATIO,
            FRONT_RIGHT_DRIVE_MOTOR,
            FRONT_RIGHT_STEER_MOTOR,
            FRONT_RIGHT_STEER_ENCODER,
            FRONT_RIGHT_STEER_OFFSET
        );

        m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            m_tab.getLayout("Back Left Module", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 0))
                    .withSize(1, 4)
                    .withPosition(4, 0),
            GEAR_RATIO,
            BACK_LEFT_DRIVE_MOTOR,
            BACK_LEFT_STEER_MOTOR,
            BACK_LEFT_STEER_ENCODER,
            BACK_LEFT_STEER_OFFSET
        );
        

        m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            m_tab.getLayout("Back Right Module", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 0))
                    .withSize(1, 4)
                    .withPosition(6, 0),
            GEAR_RATIO,
            BACK_RIGHT_DRIVE_MOTOR,
            BACK_RIGHT_STEER_MOTOR,
            BACK_RIGHT_STEER_ENCODER,
            BACK_RIGHT_STEER_OFFSET
        );
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        m_pigeon.setFusedHeading(0.0);
    }

    /**
     * Gets the current rotation from the Gyroscope
     * @return
     *  Degreess from the Pigeon (NOT ROTATION2D)
     */
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
    }

    /**
     * Sets the chassisSpeeds object in drivetrain
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public void driveAuto(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond * AUTO_DRIVE_SCALE;
        m_chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond * AUTO_DRIVE_SCALE;
        m_chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond;
        
    }

    /**
     * Sets module states of all the modules
     * @param states
     *  SwerveModuleState array, Order: FL, FR, BL, BR
     */
    public void actuateModules(SwerveModuleState[] states){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        drive(m_kinematics.toChassisSpeeds(states));
    }

    public void actuateModulesAuto(SwerveModuleState[] states){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        driveAuto(m_kinematics.toChassisSpeeds(states));
    }

    /**
     * Periodic method of Drivetrain, runs every 20ms
     */
    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        m_odometry.update(getGyroscopeRotation(), states);
        // System.out.println(getGyroscopeRotation());
        // System.out.println(getCurrentPose());
        // System.out.println(MAX_VELOCITY_METERS_PER_SECOND);
        // System.out.println(MAX_VOLTAGE);
        // System.out.println("w/o scale:" + m_chassisSpeeds.vxMetersPerSecond);
        // System.out.println("S0 m/s" + states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE);
        // System.out.println("rad/s" + m_chassisSpeeds.omegaRadiansPerSecond);
        // System.out.println("rad state [0] " + states[0].angle.getRadians());
        // System.out.println(m_chassisSpeeds.vxMetersPerSecond * AUTO_DRIVE_SCALE);
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

    /**
     * Returns the SwerveDriveKinematics object
     */
    public SwerveDriveKinematics getKinematics(){
        return m_kinematics;
    }

    /**
     * Returns the current position of the robot from the Odometry
     */
    public Pose2d getCurrentPose(){
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the Odometry to the specified Pose
     * ONLY USE IF YOU KNOW WHAT YOU ARE DOING
     * @param pose
     *  The Pose to reset the odometry to
     */
    public void resetOdometry(Pose2d pose){
        m_odometry.resetPosition(pose, getGyroscopeRotation());
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }

        return instance;
    }
}
