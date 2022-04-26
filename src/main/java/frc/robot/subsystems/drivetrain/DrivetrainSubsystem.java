// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DRIVETRAIN_PIGEON_ID;
import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.GEAR_RATIO;
import static frc.robot.Constants.MODULE_CONFIGURATION;

import java.util.Map;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import frc.lib.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import frc.lib.swervedrivespecialties.swervelib.SwerveModule;
import frc.robot.Constants;

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
    public static final double MAX_VOLTAGE = 8; //Constants.FALCON_500_FREE_SPEED / 60.0 / MODULE_CONFIGURATION.getDriveReduction() * MODULE_CONFIGURATION.getWheelDiameter() * Math.PI;
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
    private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID, "Drivetrain-CANivore");

    // These are our modules. We initialize them in the initializeMotors method.
    private SwerveModule m_frontLeftModule;
    private SwerveModule m_frontRightModule;
    private SwerveModule m_backLeftModule;
    private SwerveModule m_backRightModule;
    
    // Odometry for storing the position of the robot
    private SwerveDriveOdometry m_odometry;

    private ShuffleboardTab m_tab;
    private ShuffleboardTab m_driverTab;

    private double preciseModeScale = 1;

    private NetworkTableEntry odometryEntry;

    // ChassisSpeeds object to supply the drivetrain with (X, Y, Rotation)
    private ChassisSpeeds m_chassisSpeeds;
    private boolean lockInPlace_ = false;
    
    public final Field2d m_field = new Field2d();

    private DrivetrainSubsystem() {
        m_tab = Shuffleboard.getTab("Drivetrain");
        m_driverTab = Shuffleboard.getTab("Driver");
        m_driverTab
        .getLayout("Drivetrain Info", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withProperties(Map.of("Label position", "BOTTOM"))
        .withPosition(4, 1);
        odometryEntry = m_driverTab.add("Odometry", "not found").getEntry();

        m_chassisSpeeds  = new ChassisSpeeds(0.0, 0.0, 0.0);
        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());

        initializeMotors();       
        SmartDashboard.putData("Field", m_field);
    }


    public void initializeMotors() {
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();
        config.setCanivoreName("Drivetrain-CANivore");
        m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            // Allows you to see the current state of the module on the dashboard.
            m_tab.getLayout("Front Left Module", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 0))
                    .withSize(1, 4)
                    .withPosition(0, 0),
            // Module Config
            config,
            // L1 - L4 Change in Constants
            GEAR_RATIO,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
        );

        Timer.delay(0.06);



        m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            m_tab.getLayout("Front Right Module", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 0))
                    .withSize(1, 4)
                    .withPosition(2, 0),
            config,
            GEAR_RATIO,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR, 
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
        );
        Timer.delay(0.06);

        m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            m_tab.getLayout("Back Left Module", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 0))
                    .withSize(1, 4)
                    .withPosition(4, 0),
            config,
            GEAR_RATIO,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
        );
        Timer.delay(0.06);


        m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            m_tab.getLayout("Back Right Module", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 0))
                    .withSize(1, 4)
                    .withPosition(6, 0),
            config,
            GEAR_RATIO,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
        );
        Timer.delay(0.06);
        m_pigeon.clearStickyFaults();
    }

    public void retryMotors() {
        initializeMotors();
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        // m_pigeon.zeroGyroBiasNow();
        m_pigeon.setYaw(0.0);
        
    }

    public void setGyroscope(double deg) {
        m_pigeon.setYaw(deg);
    }

    /**
     * Gets the current rotation from the Gyroscope
     * @return
     *  Degreess from the Pigeon (NOT ROTATION2D)
     */
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
        
        
    }

    public double getGyroRoll() {
        return m_pigeon.getRoll();
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

    // Note: to get to max speed multiply by max voltage on the desaturateWheelSpeeds
    // To control speed of auto multiply by any number, 0 < x < max voltage
    // This has not been tested, most likely is completely untrue...
    // Instead use parameter of loadTrajectory()
    public void actuateModulesAuto(SwerveModuleState[] states){
        driveAuto(m_kinematics.toChassisSpeeds(states));
    }

    /**
     * Periodic method of Drivetrain, runs every 20ms
     */
    @Override
    public void periodic() {
        odometryEntry.setString(getCurrentPose().toString());
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND); 
        
        // Shuffleboard.getTab("Drivetrain").add("fl reading raw", m_frontLeftCanCoder.getAbsolutePosition());

        // System.out.println(getGyroscopeRotation());
        SmartDashboard.putNumber("Gyro", -m_pigeon.getYaw());
        m_field.setRobotPose(m_odometry.getPoseMeters());
        SmartDashboard.putNumber("roll", this.getGyroRoll());
        // if (m_frontLeftCanCoder.getLastError() != ErrorCode.OK ||
        //     m_frontRightCanCoder.getLastError() != ErrorCode.OK ||
        //     m_backLeftCanCoder.getLastError() != ErrorCode.OK ||
        //     m_backRightCanCoder.getLastError() != ErrorCode.OK) {
        //         SmartDashboard.putBoolean("Bad CanCoder Periodic", true);

        // }
        // System.out.println(getCurrentPose());
        // System.out.println(MAX_VELOCITY_METERS_PER_SECOND);
        // System.out.println(MAX_VOLTAGE);
        // System.out.println("w/o scale:" + m_chassisSpeeds.vxMetersPerSecond);
        // System.out.println("S0 m/s" + states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE);
        // System.out.println("rad/s" + m_chassisSpeeds.omegaRadiansPerSecond);
        // System.out.println("rad state [0] " + states[0].angle.getRadians());
        // System.out.println(m_chassisSpeeds.vxMetersPerSecond * AUTO_DRIVE_SCALE);
        
        if (!lockInPlace_) {
            m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
            m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
            m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
            m_backRightModule.set( states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,states[3].angle.getRadians());
        } else {
            m_frontLeftModule.set(0, -45);
            m_frontRightModule.set(0, 315);
            m_backLeftModule.set(0, 45);
            m_backRightModule.set(0, -315);
        }
        
        states[0].speedMetersPerSecond = Math.abs(m_frontLeftModule.getDriveVelocity());
       states[1].speedMetersPerSecond = Math.abs(m_frontRightModule.getDriveVelocity());
       states[2].speedMetersPerSecond = Math.abs(m_backLeftModule.getDriveVelocity());
       states[3].speedMetersPerSecond = Math.abs(m_backRightModule.getDriveVelocity());
       m_odometry.update(getGyroscopeRotation(), states);
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
        m_odometry.resetPosition(pose, pose.getRotation());
    }

    public void setPreciseMode(boolean on) {
        preciseModeScale = on ? Constants.PRECISE_MODE_SCALE : 1;
    }

    public void setLockInPlace(boolean lock) {
        lockInPlace_ = lock ? true : false;
    }

    public double getPreciseModeScale() {
        return preciseModeScale;
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }

        return instance;
    }
}
