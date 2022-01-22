package frc.robot.commands;


import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PIDContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryFollow {

    public SequentialCommandGroup getTrajectoryCommand(DrivetrainSubsystem drivetrain, PathPlannerTrajectory trajectory) {

        // Makes sure that the PID outputs values from -180 to 180 degrees
        PIDContainer.AUTO_THETA_PID.enableContinuousInput(-Math.PI, Math.PI);

        // Creates a new SwerveControllerCommand
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                // The trajectory to follow
                trajectory,
                // A method refrence for constantly getting current position of the robot
                drivetrain::getCurrentPose,
                // Getting the kinematics from the drivetrain
                drivetrain.getKinematics(),
                // Position PIDControllers from PIDContainer
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(1, 1)),
                //PIDContainer.AUTO_X_PID.getAsPidController(),
                // PIDContainer.AUTO_Y_PID.getAsPidController(),
                // PIDContainer.AUTO_THETA_PID,
                // A method refrence for setting the state of the modules
                drivetrain::actuateModulesAuto,
                // Requirment of a drivetrain subsystem
                drivetrain);

        /*
         *  Reset odometry to the starting pose of the trajectory. 
         *  This effectively transforms the trajectory to the current pose of the robot
         */
        drivetrain.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0)));
    }
}