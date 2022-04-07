
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.sql.ClientInfoStatus;
import java.time.Instant;

import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberStates;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
import frc.robot.subsystems.drivetrain.commands.auto_routines.FiveBallAuto;
import frc.robot.subsystems.drivetrain.commands.auto_routines.OneBall;
import frc.robot.subsystems.drivetrain.commands.auto_routines.ThreeBallBottomTarmac;
import frc.robot.subsystems.drivetrain.commands.auto_routines.TwoBallFender;
import frc.robot.subsystems.drivetrain.commands.auto_routines.TwoBallHanger;
import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.drivetrain.commands.auto_routines.FourBallTest;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.intake.commands.ReverseIntakeCmd;
import frc.robot.subsystems.intake.commands.RunIntakeCmd;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.AutoShot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.Rumble;


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
    // private final XboxController m_testController = new XboxController(2);


    private final DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    private final Climber climber_ = Climber.getInstance();
    private final Rumble rumble_ = Rumble.getInstance(m_driverController, m_operatorController);
    
    private final Vision m_vision = Vision.getInstance();
    // private final VisionSubsystem m_vision = VisionSubsystem.getInstance();
    // private final BionicController m_controller = new BionicController(2);

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
            () -> -modifyAxis(-m_driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * m_drivetrainSubsystem.getPreciseModeScale(),
            () -> -modifyAxis(-m_driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * m_drivetrainSubsystem.getPreciseModeScale(),
            () -> ((-modifyAxis(m_driverController.getRightX()) + m_vision.getLimelightOffset()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) * m_drivetrainSubsystem.getPreciseModeScale()
    ));
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> -modifyAxis(m_driverController.getLeftY()),
    //         () -> -modifyAxis(m_driverController.getLeftX()),
    //         () -> -modifyAxis(m_driverController.getRightX())
    // ));
        configureButtonBindings();
        configureSendableChooser();

    }

    private void configureSendableChooser() {
        m_chooser.addOption("Three Ball from Bottom of Tarmac", new ThreeBallBottomTarmac());
        m_chooser.addOption("Two Ball from Hanger Side", new TwoBallHanger(135));
        m_chooser.addOption("Fender Shot", new FenderShot());
        m_chooser.addOption("Blue One Ball Taxi", new OneBall());
        m_chooser.addOption("Five Ball Auto", new FiveBallAuto(90));
        SmartDashboard.putData(m_chooser);
    }

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
        //All these will be on the operator controller
       
        // new Button(m_operatorController::getBackButton).whenPressed(() -> climber_.setElevatorGains(1, 0, 0, 0)); //up
        // new Button(m_operatorController::getStartButton).whenPressed(() -> climber_.sete//down
        // new Button(m_driverController::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
        new Button(m_driverController::getStartButton);
        // .toggleWhenPressed(new InstantCommand(() -> m_vision.switchCamera("Climber Camera"))
        // .andThen(new InstantCommand(() -> m_vision.switchCamera("Front Camera"))));
    
    // new Button(m_controller::getBButton).whenPressed(m_VisionSubsystem::getDistance);
    // Switch Pipelines
    new Button(m_driverController::getRightBumper)

                .whenHeld(new InstantCommand(() -> m_drivetrainSubsystem.setPreciseMode(true)))
                .whenReleased(new InstantCommand(() -> m_drivetrainSubsystem.setPreciseMode(false)));
    new Button(m_driverController::getLeftBumper)
                .whenHeld(new InstantCommand(() -> m_drivetrainSubsystem.setLockInPlace(true)))
                .whenReleased(new InstantCommand(() -> m_drivetrainSubsystem.setLockInPlace(false)));
    // new Button(m_driverController::getRightStickButton).whenPressed(m_VisionSubsystem::setPipeline);
    

    //Fender Shot Angles
    // new Button(m_driverController::getAButton).whenPressed(new SnapToAngle(m_driverController, 201, m_drivetrainSubsystem));
    // new Button(m_driverController::getBButton).whenPressed(new SnapToAngle(m_driverController, 111, m_drivetrainSubsystem));
    // new Button(m_driverController::getXButton).whenPressed(new SnapToAngle(m_driverController, 291, m_drivetrainSubsystem));
    // new Button(m_driverController::getYButton).whenPressed(new SnapToAngle(m_driverController, 21, m_drivetrainSubsystem));
    // new Button(() -> (m_driverController.getPOV()  != -1)).whenPressed(new SnapToAngle(m_driverController, m_drivetrainSubsystem), false);

    // new Button(m_controller::getLeftBumper).whenPressed(new RunCommand(() -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(() -> m_controller.getLeftBumperReleased()).andThen(new RunCommand(() -> m_VisionSubsystem.endRumble(m_controller))));
    // new Button(m_controller::getLeftBumper).whenPressed(new RunCommand(() -> m_VisionSubsystem.checkRumble(m_controller, true)).withInterrupt(m_controller::getLeftBumperReleased));
    // new Button(m_controller::getRightBumper).whileActiveContinuous(new RunCommand( () -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(m_controller::getRightBumper));
    // new Button(m_controller::getLeftBumper).whileActiveContinuous(new RunCommand( () -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(m_controller::getLeftBumper));
    //.whenHeld(getLimelightCommand()
    //.alongWith(new RunCommand(() -> m_VisionSubsystem.checkRumble(m_controller)).withInterrupt(m_controller::getLeftBumperPressed)));


    // Shoot the shot
    new Trigger(() -> (Math.abs(m_driverController.getRightTriggerAxis()) > 0.7))
                .whenActive(() -> { m_intakeSubsystem.shoot(); } )
                .whenInactive(() -> { m_intakeSubsystem.stopIntake();} );

    new Trigger(() -> (Math.abs(m_driverController.getLeftTriggerAxis())) > 0.7)
        .whenActive(new AutoShot(m_vision, m_shooterSubsystem, m_hoodSubsystem, () -> m_driverController.getRightTriggerAxis() > 0.7))
        .whenInactive(new InstantCommand(() -> m_vision.setLimelightOffset(0), m_vision)
            .andThen(new InstantCommand(() -> m_shooterSubsystem.setGoalStatic(0.0, false))));
        /////////////////////////////////
        ///      Operator Buttons     ///
        /////////////////////////////////           
        // Fender shot
        // new ConditionalCommand(() -> {m_operatorController.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);}, () -> {m_operatorController.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);}, m_shooterSubsystem::spunUp);
        // new ConditionalCommand(() -> {m_operatorController.setRumble(RumbleType.kRightRumble, 1.0); m_operator.setRumble(RumbleType.kLeftRumble, 1.0); }, () -> { m_operatorController.setRumble(RumbleType.kRightRumble, 1.0); m_operator.setRumble(RumbleType.kLeftRumble, 1.0); }, m_shooterSubsystem::spunUp);
        new Button(m_operatorController::getXButton).whenPressed(m_shooterSubsystem.setLowGoalCommand(Constants.kFenderLowShotVelocity)
        .alongWith(new InstantCommand(() -> m_hoodSubsystem.setHoodAngle(21))));
        // new Button(m_operatorController::getAButton).whenPressed(() -> { m_shooterSubsystem.setVelocityGoal(Constants.kFenderShotVelocity, false);
        new Button(m_operatorController::getAButton).whenPressed(m_shooterSubsystem.setGoalCommand(Constants.kFenderShotVelocity)
        .alongWith(new InstantCommand(() -> m_hoodSubsystem.setHoodAngle(13))));
        new Trigger(() -> m_operatorController.getPOV() == 180).whenActive(() -> {m_hoodSubsystem.zeroHood();});
        
        // new Trigger(() -> m_operatorController.getPOV() == 90).whenActive(m_shooterSubsystem.setGoalCommand(Constants.kWallShotVelocity));


        // Limelight shot: Stays the same, spins up based on limelight feedback but doesn't shoot
        // new Button(m_operatorController::getXButton).whenPressed(new LimelightShoot());
        // Cancel a spin up
        new Button(m_operatorController::getBButton).whenPressed(() -> { m_shooterSubsystem.stop(); } );
        new Button(m_operatorController::getYButton).whenPressed(() -> climber_.setState(ClimberStates.PREPARE_HIGH));

        

        
    // new Trigger(() -> (Math.abs(m_operatorController.getRightTriggerAxis()) > 0.1))
    // .whenActive(climber_.RetractClimber(m_operatorController.getRightTriggerAxis()));
    

 // Run intake: Operator right trigger
        new Trigger(() -> (Math.abs(m_operatorController.getRightTriggerAxis()) > 0.7))
            .whenActive(m_intakeSubsystem::intake)
            .whenInactive(m_intakeSubsystem::stopIntake);

        // Reverse intake: Operator left trigger
        new Trigger(() -> (Math.abs(m_operatorController.getLeftTriggerAxis()) > 0.7))
            .whenActive(m_intakeSubsystem::reverseIntake)
            .whenInactive(m_intakeSubsystem::stopIntake);

        new Trigger(() -> m_operatorController.getPOV() == 270).whenActive(m_intake::intakeZero);


        new Button(m_operatorController::getBackButton).whenPressed(() -> climber_.setState(ClimberStates.CALIBRATE));
        new Button(m_operatorController::getStartButton).whenPressed(() -> climber_.setState(ClimberStates.MID_ALIGN));
        // new Button(m_operatorController::getLeftBumper).whenPressed(climber_::StartRoutine);
        // new Button(m_operatorController::getRightBumper).whenPressed(climber_::StopRoutine); //Only do in case of emergency, has to be manually reset :(
        //driver controller
        new Button(m_operatorController::getLeftBumper).whenPressed(() -> climber_.setState(ClimberStates.RETRACTION));

        new Button(m_operatorController::getRightBumper).whenPressed(() -> climber_.setState(ClimberStates.HIGHER_CLIMB));
        // new Button(m_operatorController::getRightStickButton).whenPressed(climber_.ExtendClimberHigh());
    }
    

    public void temp() {
        SmartDashboard.putNumber("POV", m_driverController.getPOV());
    }

    

    // new Button(m_operatorController::getXButton).whenPressed(m_VisionSubsystem::setPipelineOne);

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
