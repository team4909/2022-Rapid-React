package frc.robot.subsystems.drivetrain.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class TrajectoryFollow extends CommandBase {

    private String pathName;

    public TrajectoryFollow() {
        //TODO add default stationary path
        this.pathName = "";
    }

    public TrajectoryFollow(String pathName) {
        this.pathName = pathName;
    }

    @Override
    public void initialize() {
        System.out.println("Trajectory begun");

        PathPlannerTrajectory trajectory = null;
        try {
            trajectory = PathPlanner.loadPath(pathName, 2.9, 4);
        } catch (Exception e) {
            e.printStackTrace();
        }

        PathPlannerState finalAngle = ((PathPlannerState) trajectory.getEndState());
        new SwerveControllerCommand(trajectory, 
        DrivetrainSubsystem.getInstance()::getCurrentPose, 
        DrivetrainSubsystem.getInstance().getKinematics(), 
        new PIDController(1.5, 0, 0), 
        new PIDController(1.5, 0, 0),
        new ProfiledPIDController(1.5, 0, 0, new TrapezoidProfile.Constraints(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Math.pow(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 2))),
        () -> finalAngle.holonomicRotation,
        DrivetrainSubsystem.getInstance()::actuateModulesAuto, 
        DrivetrainSubsystem.getInstance()).andThen(() -> DrivetrainSubsystem.getInstance().drive(new ChassisSpeeds(0.0, 0.0, 0.0))).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
