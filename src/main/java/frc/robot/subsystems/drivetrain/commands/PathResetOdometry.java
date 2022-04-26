package frc.robot.subsystems.drivetrain.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class PathResetOdometry extends CommandBase {

    PathPlannerTrajectory trajectory = null;

    public PathResetOdometry(String pathName) {
        try {
            trajectory = PathPlanner.loadPath(pathName, 8, 5);
        } catch (NullPointerException e) {
            //If you spell the path name wrong stay still.
            trajectory = PathPlanner.loadPath("Stay Still", 8, 5);
        }
        
    }

    public PathResetOdometry(String pathName, double offset) {
        try {
            trajectory = PathPlanner.loadPath(pathName, 8, 5);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void initialize() {
        Pose2d currentPose  = DrivetrainSubsystem.getInstance().getCurrentPose();
        Pose2d initialPose = trajectory.getInitialPose();
        Rotation2d offsetRot = initialPose.getRotation().minus(currentPose.getRotation());
        
        Pose2d offsetPose = new Pose2d(
            initialPose.getX(),
            initialPose.getY(),
            offsetRot
        );

        // DrivetrainSubsystem.getInstance().setGyroscope(offsetRot.getDegrees());
        DrivetrainSubsystem.getInstance().setGyroscope(initialPose.getRotation().getDegrees());
        
        DrivetrainSubsystem.getInstance().resetOdometry(offsetPose);
        
    }
    
}