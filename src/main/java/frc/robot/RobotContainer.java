
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.time.Instant;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.climber.Climber;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.Taxi;
import frc.robot.subsystems.drivetrain.commands.AlignWithGoal;
import frc.robot.subsystems.drivetrain.commands.DefaultDriveCommand;
import frc.robot.subsystems.drivetrain.commands.SnapToAngle;
import frc.robot.subsystems.drivetrain.commands.auto_routines.FenderShot;
// import frc.robot.subsystems.drivetrain.commands.auto_routines.FourBallTest;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.intake.commands.ReverseIntakeCmd;
import frc.robot.subsystems.intake.commands.RunIntakeCmd;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.LimelightShootCmd;
import frc.robot.subsystems.shooter.commands.ShootCmd;
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
    // private final Climber climber_ = Climber.getInstance();
    
    private final VisionSubsystem m_VisionSubsystem = VisionSubsystem.getInstance();
    // private final BionicController m_controller = new BionicController(2);

    // private final PowerDistribution PDH = new PowerDistribution(0, ModuleType.kRev);

    private final Shooter m_shooterSubsystem = Shooter.getInstance();
    private final IntakeFeeder m_intakeSubsystem = IntakeFeeder.getInstance();
    
    private final XboxController m_driverController = new XboxController(0);
    private final XboxController m_operatorController = new XboxController(1);

 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // PDH.clearStickyFaults();
    // Create the driver tab
    Shuffleboard.getTab("Driver");

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * m_drivetrainSubsystem.getPreciseModeScale(),
            () -> -modifyAxis(m_driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * m_drivetrainSubsystem.getPreciseModeScale(),
            () -> (-modifyAxis(m_driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND + m_VisionSubsystem.getLimelightOffset()) * m_drivetrainSubsystem.getPreciseModeScale()
    ));

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
        ///////////////////////////////
        ///      Driver Buttons     ///
        ///////////////////////////////

        // Back button zeros the gyroscope
        // new Button(m_controller::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope, m_drivetrainSubsystem);
        //All these will be on the operator controller
//         new Button(m_operator::getBackButton).whenPressed(climber_::RaiseClimber);
//         new Button(m_operator::getStartButton).whenPressed(climber_::LowerClimber);
//         new Button(m_operator::getLeftBumper).whenPressed(climber_::StartRoutine);
//         new Button(m_operator::getRightBumper).whenPressed(climber_::StopRoutine); //Only do in case of emergency, has to be manually reset :(
        //driver controller
        // new Button(m_driverController::getLeftBumper).whenPressed(new InstantCommand(() -> climber_.setElevatorGoal(26)));
        // new Button(m_driverController::getRightBumper).whenPressed(new InstantCommand(() -> climber_.setElevatorGoal(0.76)));
        new Button(m_driverController::getBackButton)
                // No requirements because we don't need to interrupt anything
                .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    // new Button(m_controller::getBButton).whenPressed(m_VisionSubsystem::getDistance);
    // Switch Pipelines
    new Button(m_driverController::getXButton)
    .whenHeld(new InstantCommand(() -> m_drivetrainSubsystem.setPreciseMode(true)))
    .whenReleased(new InstantCommand(() -> m_drivetrainSubsystem.setPreciseMode(false)));
    new Button(m_driverController::getRightStickButton).whenPressed(m_VisionSubsystem::setPipeline);
    new Button(m_driverController::getYButton).whenHeld(new RunCommand(m_VisionSubsystem::setLimelightOffset))
    .whenReleased(() -> m_VisionSubsystem.setLimelightOffset(0));

    new Trigger(() -> (m_driverController.getPOV()  == 90)).whenActive(new SnapToAngle(90d, m_drivetrainSubsystem).withTimeout(2));
    // new Button(m_controller::getLeftBumper).whenPressed(new RunCommand(() -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(() -> m_controller.getLeftBumperReleased()).andThen(new RunCommand(() -> m_VisionSubsystem.endRumble(m_controller))));
    // new Button(m_controller::getLeftBumper).whenPressed(new RunCommand(() -> m_VisionSubsystem.checkRumble(m_controller, true)).withInterrupt(m_controller::getLeftBumperReleased));
    // new Button(m_controller::getRightBumper).whileActiveContinuous(new RunCommand( () -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(m_controller::getRightBumper));
    // new Button(m_controller::getLeftBumper).whileActiveContinuous(new RunCommand( () -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(m_controller::getLeftBumper));
    //.whenHeld(getLimelightCommand()
    //.alongWith(new RunCommand(() -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(m_controller::getLeftBumperPressed)));
        // Shoot the shot
        new Trigger(() -> (Math.abs(m_driverController.getRightTriggerAxis()) > 0.7))
                    .whenActive(() -> { m_intakeSubsystem.shoot(); } )
                    .whenInactive(() -> { m_intakeSubsystem.stopIntake(); m_shooterSubsystem.stop(); } );


        /////////////////////////////////
        ///      Operator Buttons     ///
        /////////////////////////////////           
        // Fender shot
        // new ConditionalCommand(() -> {m_operatorController.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);}, () -> {m_operatorController.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);}, m_shooterSubsystem::spunUp);
        // new ConditionalCommand(() -> {m_operatorController.setRumble(RumbleType.kRightRumble, 1.0); m_operator.setRumble(RumbleType.kLeftRumble, 1.0); }, () -> { m_operatorController.setRumble(RumbleType.kRightRumble, 1.0); m_operator.setRumble(RumbleType.kLeftRumble, 1.0); }, m_shooterSubsystem::spunUp);
        new Button(m_operatorController::getAButton).whenPressed(() -> { m_shooterSubsystem.setVelocityGoal(Constants.kFenderShotVelocity, false);});
        new Trigger(() -> m_operatorController.getPOV() == 90).whenActive(() -> { m_shooterSubsystem.setVelocityGoal(Constants.kLongShotVelocity, true);});

        // Limelight shot: Stays the same, spins up based on limelight feedback but doesn't shoot
        new Button(m_operatorController::getXButton).whenPressed(new LimelightShootCmd());
        // Cancel a spin up
        new Button(m_operatorController::getBButton).whenPressed(() -> { m_shooterSubsystem.stop(); } );
        new Button(m_operatorController::getYButton).whenPressed(new RunCommand(m_intakeSubsystem::compressBalls).withTimeout(1));

        // new Trigger().whenActive(new RunIntakeCmd())

        // Run intake: Operator right trigger
        new Trigger(() -> (Math.abs(m_operatorController.getRightTriggerAxis()) > 0.7))
        .whenActive(m_intakeSubsystem::intake)
        .whenInactive(m_intakeSubsystem::stopIntake);

        // Reverse intake: Operator left trigger
        new Trigger(() -> (Math.abs(m_operatorController.getLeftTriggerAxis()) > 0.7))
        .whenActive(m_intakeSubsystem::reverseIntake)
        .whenInactive(m_intakeSubsystem::stopIntake);
    }
    

    // new Button(m_operatorController::getXButton).whenPressed(m_VisionSubsystem::setPipelineOne);

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *%%
   * @return the command to run in autonomous
 * @throws IOException
   */
    public Command getAutonomousCommand() {

        return new FenderShot();
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
        value = deadband(value, 0.1);

        // Square the axis
        value = Math.copySign(value * value, value);


        return value;
    }


    
}
