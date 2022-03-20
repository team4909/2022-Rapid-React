package frc.robot.subsystems.drivetrain.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class PathResetOdometry extends CommandBase {

	PathPlannerTrajectory trajectory = null;
	private double offset_;

	public PathResetOdometry(String pathName) {
		try {
			trajectory = PathPlanner.loadPath(pathName, 8, 5);
		} catch (Exception e) {
			e.printStackTrace();
		}
		offset_ = 0;

	}

	public PathResetOdometry(String pathName, double offset) {
		try {
			trajectory = PathPlanner.loadPath(pathName, 8, 5);
		} catch (Exception e) {
			e.printStackTrace();
		}
		double offset_ = offset;
	}

	@Override
	public void initialize() {
		Pose2d initialPose = trajectory.getInitialPose();
		Pose2d offsetPose = new Pose2d(
				initialPose.getX(),
				initialPose.getY(),
				new Rotation2d(initialPose.getRotation().getDegrees() + offset_)
		);
		DrivetrainSubsystem.getInstance().resetOdometry(offsetPose);

	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}

}