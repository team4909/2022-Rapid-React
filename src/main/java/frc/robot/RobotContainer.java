
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.commands.AlignWithGoal;
import frc.robot.subsystems.drivetrain.commands.DefaultDriveCommand;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.drivetrain.commands.auto_routines.FenderShot;
import frc.robot.subsystems.drivetrain.commands.auto_routines.FourBallTest;
import frc.robot.subsystems.vision.LimeLight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.BionicController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    private final VisionSubsystem m_VisionSubsystem = VisionSubsystem.getInstance();
    private final BionicController m_controller = new BionicController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    // UNCOMMENT 63-68 IF YOU NEED TO DRIVE ROBOT
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));


        // Configure the button bindings
        configureButtonBindings();


    }

    public Command getLimelightCommand() {
        return new AlignWithGoal(m_drivetrainSubsystem, () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Back button zeros the gyroscope
        new Button(m_controller::getBackButton)
                // No requirements because we don't need to interrupt anything
                .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    new Button(m_controller::getBButton).whenPressed(m_VisionSubsystem::getDistance);
    // Switch Pipelines
    new Button(m_controller::getXButton).whenPressed(m_VisionSubsystem::setPipeline);
    // new Button(m_controller::getLeftBumper).whenPressed(new RunCommand(() -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(() -> m_controller.getLeftBumperReleased()).andThen(new RunCommand(() -> m_VisionSubsystem.endRumble(m_controller))));
    // new Button(m_controller::getLeftBumper).whenPressed(new RunCommand(() -> m_VisionSubsystem.checkRumble(m_controller, true)).withInterrupt(m_controller::getLeftBumperReleased));
    new Button(m_controller::getRightBumper).whileActiveContinuous(new RunCommand( () -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(m_controller::getRightBumper));
    new Button(m_controller::getLeftBumper).whileActiveContinuous(new RunCommand( () -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(m_controller::getLeftBumper));
    //.whenHeld(getLimelightCommand()
    //.alongWith(new RunCommand(() -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(m_controller::getLeftBumperPressed)));
    }

    // new Button(m_operatorController::getXButton).whenPressed(m_VisionSubsystem::setPipelineOne);

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *%%
   * @return the command to run in autonomous
 * @throws IOException
   */
    public Command getAutonomousCommand() {

        return new FourBallTest();
    }

    // public PathPlannerTrajectory getTrajectory(){
    //     return new LoadPath("simple").getTrajectory();
    // }

    /**
     * Deadbands a value based on the given constraints
     * @param value
     *    The raw value to deadband
     * @param deadband
     *    The deadband constraint
     * @return
     *    Deadbanded value
     */
    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Modifies the value passed in, deadbanded, and squared
     * @param value
     *    The raw value to be modified
     * @return
     *    The modified value
     */
    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.025);

        // Square the axis
        value = Math.copySign(value * value, value);


        return value;
    }

    
}
