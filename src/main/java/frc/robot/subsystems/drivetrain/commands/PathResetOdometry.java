package frc.robot.subsystems.drivetrain.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class PathResetOdometry extends CommandBase {

    PathPlannerTrajectory trajectory = null;

    public PathResetOdometry(String pathName) {
        try {
            trajectory = PathPlanner.loadPath(pathName, 8, 5);
        } catch (Exception e) {
            e.printStackTrace();
        }
        
    }

    @Override
    public void initialize() {
        DrivetrainSubsystem.getInstance().resetOdometry(trajectory.getInitialPose());
    
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