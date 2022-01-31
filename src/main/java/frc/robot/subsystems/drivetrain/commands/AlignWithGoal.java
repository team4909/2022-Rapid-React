package frc.robot.subsystems.drivetrain.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class AlignWithGoal extends CommandBase {

    DrivetrainSubsystem dt = DrivetrainSubsystem.getInstance();


    public AlignWithGoal() {

    }

    @Override
    public void execute() {
        var turnTraj = TrajectoryGenerator.generateTrajectory(
            DrivetrainSubsystem.getInstance().getCurrentPose(),
            new ArrayList<Translation2d>(),
            new Pose2d(dt.getCurrentPose().getX(), dt.getCurrentPose().getY(), Rotation2d.fromDegrees(90)),
            new TrajectoryConfig(Constants.T_MAX_VEL, Constants.T_MAX_VEL));

        new TrajectoryFollow(turnTraj);
    }

}
