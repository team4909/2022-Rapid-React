package frc.robot.subsystems.drivetrain.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class TrajectoryFollow extends CommandBase {

    private String m_pathName;


    public TrajectoryFollow() {
        //TODO add default stationary path
        m_pathName = "";
    }

    public TrajectoryFollow(String pathName) {
        m_pathName = pathName;
    }

    @Override
    public void initialize() {
        System.out.println("Trajectory begun");

        PathPlannerTrajectory trajectory = null;
        try {
            trajectory = PathPlanner.loadPath(m_pathName, 4.9, 4);
        } catch (Exception e) {
            e.printStackTrace();
        }

        ProfiledPIDController thetaController = new ProfiledPIDController(1.5, 0, 0, new TrapezoidProfile.Constraints(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Math.pow(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 2)));
        thetaController.enableContinuousInput(-180, 180);

        new PPSwerveControllerCommand(trajectory, 
        DrivetrainSubsystem.getInstance()::getCurrentPose,
        DrivetrainSubsystem.getInstance().getKinematics(), 
        new PIDController(3, 0, 0), 
        new PIDController(3, 0, 0),
        thetaController, 
        DrivetrainSubsystem.getInstance()::actuateModulesAuto, 
        DrivetrainSubsystem.getInstance()).andThen(() -> DrivetrainSubsystem.getInstance().drive(new ChassisSpeeds(0.0, 0.0, 0.0))).schedule();
        }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}

/* OLD PATHFOLLOWING COMMAND
PathPlannerState finalAngle = ((PathPlannerState) trajectory.getEndState());

        // new SwerveControllerCommand(trajectory, 
        // DrivetrainSubsystem.getInstance()::getCurrentPose, 
        // DrivetrainSubsystem.getInstance().getKinematics(), 
        // new PIDController(3, 0, 0), 
        // new PIDController(3, 0, 0),
        // thetaController,
        // () -> finalAngle.holonomicRotation,
        // DrivetrainSubsystem.getInstance()::actuateModulesAuto, 
        // DrivetrainSubsystem.getInstance()).andThen(() -> DrivetrainSubsystem.getInstance().drive(new ChassisSpeeds(0.0, 0.0, 0.0))).schedule();
    
 */