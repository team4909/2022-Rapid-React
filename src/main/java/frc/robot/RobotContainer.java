// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberStates;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.commands.DefaultDriveCommand;
import frc.robot.subsystems.drivetrain.commands.auto_routines.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.AutoShot;
import frc.robot.subsystems.vision.Vision;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private final XboxController m_driverController = new XboxController(0);
    private final XboxController m_operatorController = new XboxController(1);

    private final DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    private final Climber climber_ = Climber.getInstance();
    
    private final Vision m_vision = Vision.getInstance();

    private final PowerDistribution PDH;

    private final Hood m_hoodSubsystem  = Hood.getInstance();
    private final Shooter m_shooterSubsystem = Shooter.getInstance();
    private final IntakeFeeder m_intakeSubsystem = IntakeFeeder.getInstance();
    private final Intake m_intake = Intake.getInstance();

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // PDH.clearStickyFaults();

    //Allows access to photon vision client when tethered via USB cable
    PortForwarder.add(5800, "gloworm.local", 5800);
    
    // Create the driver tab
    Shuffleboard.getTab("Driver");
    PDH = new PowerDistribution(1, ModuleType.kRev);
    PDH.clearStickyFaults(); 
    LiveWindow.disableTelemetry(PDH);
    System.out.println(PDH.getCurrent(1));

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * m_drivetrainSubsystem.getPreciseModeScale(),
            () -> -modifyAxis(m_driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * m_drivetrainSubsystem.getPreciseModeScale(),
            () -> ((-modifyAxis(m_driverController.getRightX()) + m_vision.getLimelightOffset()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) * m_drivetrainSubsystem.getPreciseModeScale()
    ));

    configureButtonBindings();
    configureSendableChooser();

    }

    private void configureSendableChooser() {
        // m_chooser.addOption("Three Ball from Bottom of Tarmac", new ThreeBallBottomTarmac());
        m_chooser.addOption("Five Ball Auto", new FiveBallAuto());
        m_chooser.addOption("One Ball", new OneBall());
        m_chooser.addOption("Two Ball", new TwoBall());
        m_chooser.addOption("Two Ball Alt", new TwoBallAlt());
        m_chooser.addOption("One Ball Disrupt", new OneBallDisrupt());
        m_chooser.addOption("Two Ball Disrupt", new TwoBallDisrupt());
        SmartDashboard.putData(m_chooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        //#region Driver Controls
        new Button(m_driverController::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
        new Button(m_driverController::getStartButton);

        new Button(m_driverController::getRightBumper)
                    .whenHeld(new InstantCommand(() -> m_drivetrainSubsystem.setPreciseMode(true)))
                    .whenReleased(new InstantCommand(() -> m_drivetrainSubsystem.setPreciseMode(false)));

        new Button(m_driverController::getLeftBumper)
                    .whenHeld(new InstantCommand(() -> m_drivetrainSubsystem.setLockInPlace(true)))
                    .whenReleased(new InstantCommand(() -> m_drivetrainSubsystem.setLockInPlace(false)));

        new Trigger(() -> (Math.abs(m_driverController.getRightTriggerAxis()) > 0.7))
                    .whenActive(() -> { m_intakeSubsystem.shoot(); } )
                    .whenInactive(() -> { m_intakeSubsystem.stopIntake();} );

        new Trigger(() -> (Math.abs(m_driverController.getLeftTriggerAxis())) > 0.7)
            .whenActive(new AutoShot(m_vision, m_shooterSubsystem, m_hoodSubsystem, () -> m_driverController.getRightTriggerAxis() > 0.7))
            .whenInactive(new InstantCommand(() -> m_vision.setLimelightOffset(0), m_vision)
                .andThen(new InstantCommand(() -> m_shooterSubsystem.setGoalStatic(0.0, false))));
        //#endregion

        //#region Operator Controls
        new Button(m_operatorController::getXButton).whenPressed(m_shooterSubsystem.setLowGoalCommand(Constants.kFenderLowShotVelocity)
        .alongWith(new InstantCommand(() -> m_hoodSubsystem.setHoodAngle(21))));
        new Button(m_operatorController::getAButton).whenPressed(m_shooterSubsystem.setGoalCommand(Constants.kFenderShotVelocity)
        .alongWith(new InstantCommand(() -> m_hoodSubsystem.setHoodAngle(Constants.kFenderShotHoodAngle))));
        new Trigger(() -> m_operatorController.getPOV() == 180).whenActive(() -> {m_hoodSubsystem.zeroHood();});

        new Button(m_operatorController::getBButton).whenPressed(() -> { m_shooterSubsystem.stop(); } );

        new Trigger(() -> (Math.abs(m_operatorController.getRightTriggerAxis()) > 0.7))
            .whenActive(m_intakeSubsystem::intake)
            .whenInactive(m_intakeSubsystem::stopIntake);

        new Trigger(() -> (Math.abs(m_operatorController.getLeftTriggerAxis()) > 0.7))
            .whenActive(m_intakeSubsystem::reverseIntake)
            .whenInactive(m_intakeSubsystem::stopIntake);

        new Trigger(() -> m_operatorController.getPOV() == 270).whenActive(m_intake::intakeZero);

        new Button(m_operatorController::getBackButton).whenPressed(() -> climber_.setState(ClimberStates.CALIBRATE));
        new Button(m_operatorController::getStartButton).whenPressed(() -> climber_.setState(ClimberStates.MID_ALIGN));
        new Button(m_operatorController::getLeftBumper).whenPressed(() -> climber_.setState(ClimberStates.RETRACTION));
        new Button(m_operatorController::getYButton).whenPressed(() -> climber_.setState(ClimberStates.PREPARE_HIGH));
        new Button(m_operatorController::getRightBumper).whenPressed(() -> climber_.setState(ClimberStates.HIGHER_CLIMB));
        //#endregion

    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   *%%
   * @return the command to run in autonomous
 * @throws IOException
   */
    public Command getAutonomousCommand() {

        // return new AutoTest();
        return m_chooser.getSelected();
    }

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
     * 
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
