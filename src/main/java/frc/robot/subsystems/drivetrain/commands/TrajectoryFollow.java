package frc.robot.subsystems.drivetrain.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class TrajectoryFollow extends CommandBase {

    private String m_pathName;
    private PathPlannerTrajectory m_trajectory = null;
    private double m_timeout = DriveConstants.T_DEFAULT_TIMEOUT;
    private Timer timer = new Timer();

    public TrajectoryFollow() {
        m_pathName = "Stay Still";
    }

    public TrajectoryFollow(String pathName) {
        m_pathName = pathName;
    }

    public TrajectoryFollow(Trajectory traj) {
        m_trajectory = (PathPlannerTrajectory) traj;
    }

    public TrajectoryFollow(Pair<String, Double> traj) {
        m_pathName = traj.getFirst();
        m_timeout = traj.getSecond();
    }

    @Override
    public void initialize() {
        System.out.println("Trajectory begun");
        timer.start();

        if (m_trajectory == null) {
            try {
                m_trajectory = PathPlanner.loadPath(m_pathName, DriveConstants.T_MAX_VEL, DriveConstants.T_MAX_ACCEL); //2.9, 3
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        DrivetrainSubsystem.getInstance().m_field.getObject("traj").setTrajectory(m_trajectory);

        ProfiledPIDController thetaController = new ProfiledPIDController(6, 0, 0,
                new TrapezoidProfile.Constraints(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        Math.pow(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 2)));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        new PPSwerveControllerCommand(m_trajectory,
                DrivetrainSubsystem.getInstance()::getCurrentPose,
                DrivetrainSubsystem.getInstance().getKinematics(),
                new PIDController(6, 0, 0),
                new PIDController(6, 0, 0),
                thetaController,
                DrivetrainSubsystem.getInstance()::actuateModulesAuto,
                DrivetrainSubsystem.getInstance())
                        .andThen(() -> DrivetrainSubsystem.getInstance().drive(new ChassisSpeeds(0.0, 0.0, 0.0)))
                        .schedule();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(m_timeout);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        super.end(interrupted);
    }

}
