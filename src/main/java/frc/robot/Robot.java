// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private final DrivetrainSubsystem drivetrainSubsystem_ = DrivetrainSubsystem.getInstance();
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Compressor m_compressor;

  // private Command llCommand;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /**
     * Instantiate our RobotContainer.  This will perform all our button bindings, and put our
     * autonomous chooser on the dashboard.
     */

    SmartDashboard.putBoolean("Align", false);
    SmartDashboard.putNumber("x_P", 1);
    SmartDashboard.putNumber("x_I", 0);
    SmartDashboard.putNumber("x_D", 0);
    m_robotContainer = new RobotContainer();

    m_compressor = new Compressor(PneumaticsModuleType.REVPH);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /**
     * Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
     * commands, running already-scheduled commands, removing finished or interrupted commands,
     * and running subsystem periodic() methods.  This must be called from the robot's periodic
     * block in order for anything in the Command-based framework to work.
     */
    CommandScheduler.getInstance().run();


    // if (smartdashboard_button == true) {
    // new AlignWithGoal();
    // }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    System.out.println("Auto Starting");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //System.out.println(RobotContainer.trajectory.);
  }

  @Override
  public void teleopInit() {
    /**
     *  This makes sure that the autonomous stops running when
     *  teleop starts running. If you want the autonomous to
     *  continue until interrupted by another command, remove
     *  this line or comment it out.
     */
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // llCommand = m_robotContainer.getLimelightCommand();

  }

  

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if (SmartDashboard.getBoolean("Align", false) == true) {
    //   if (!llCommand.isScheduled()) {
    //     System.out.println("ll scheudlaed");
    //     llCommand.schedule();
    //   }

    // } else {
    //   if (llCommand.isScheduled()) {
    //     llCommand.cancel();
    //   }
    // }

    // m_compressor.enableAnalog(90, 120);
  


  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
