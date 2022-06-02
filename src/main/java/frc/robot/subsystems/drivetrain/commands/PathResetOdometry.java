package frc.robot.subsystems.drivetrain.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class PathResetOdometry extends CommandBase {

    PathPlannerTrajectory trajectory = null;

    public PathResetOdometry(String pathName) {
        try {
            trajectory = PathPlanner.loadPath(pathName, DriveConstants.T_MAX_VEL, DriveConstants.T_MAX_ACCEL);
        } catch (NullPointerException e) {
            trajectory = PathPlanner.loadPath("Stay Still", DriveConstants.T_MAX_VEL, DriveConstants.T_MAX_ACCEL);
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

        DrivetrainSubsystem.getInstance().setGyroscope(initialPose.getRotation().getDegrees());
        
        DrivetrainSubsystem.getInstance().resetOdometry(offsetPose);
        
    }

   public boolean isFinished() { return true; }
    
}