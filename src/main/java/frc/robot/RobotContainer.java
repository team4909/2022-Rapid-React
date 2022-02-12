
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.commands.AlignWithGoal;
import frc.robot.subsystems.drivetrain.commands.DefaultDriveCommand;

// import frc.robot.subsystems.drivetrain.commands.auto_routines.FourBallTest;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.intake.commands.ReverseIntakeCmd;
import frc.robot.subsystems.intake.commands.RunIntakeCmd;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.LimelightShootCmd;
import frc.robot.subsystems.shooter.commands.ShootCmd;
import frc.robot.subsystems.vision.LimeLight;
import frc.robot.subsystems.vision.VisionSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private final DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    
    private final VisionSubsystem m_VisionSubsystem = VisionSubsystem.getInstance();

    private final Shooter m_shooterSubsystem = Shooter.getInstance();
    private final IntakeFeeder m_intakeSubsystem = IntakeFeeder.getInstance();
    
    // private final XboxController m_driverController = new XboxController(0);
    private final XboxController m_operatorController = new XboxController(1);

 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> -modifyAxis(m_driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));

        // Configure the button bindings
        configureButtonBindings();


    }

    // public Command getLimelightCommand() {
    //     return new AlignWithGoal(m_drivetrainSubsystem, () -> -modifyAxis(m_driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis(m_driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, () -> -modifyAxis(m_driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    // }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Back button zeros the gyroscope
        // new Button(m_driverController::getBackButton)
        //         // No requirements because we don't need to interrupt anything
        //         .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
        
        // TODO No idea if this is how we are planning on doing buttons
        // But here are the mappings we can move to another structure later
        // Fender shot
        new Button(m_operatorController::getAButton).whenHeld(new ShootCmd(Constants.kFenderShotVelocity, true));
        // // Tarmac shot
        // new Button(m_driverController::getBButton).whenPressed(new ShootCmd(Constants.kTarmacShotVelocity));
        // // Limelight shot
        // new Button(m_driverController::getXButton).whenPressed(new LimelightShootCmd());



        // Run intake 
        new Button(m_operatorController::getRightBumper).whenHeld(new RunIntakeCmd());

        // Reverse intake
        new Button(m_operatorController::getLeftBumper).whenHeld(new ReverseIntakeCmd());

    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
 * @throws IOException
   */
    public Command getAutonomousCommand() {

        return null; //new FourBallTest();
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
