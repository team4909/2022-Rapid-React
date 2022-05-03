package frc.robot.subsystems.shooter.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShot extends CommandBase {

    private final Vision m_vision;
    private final Shooter m_shooter;
    private final Hood m_hood;
    private final BooleanSupplier m_shooting;

    private static double m_distanceSeen = 0;

    public AutoShot(Vision v, Shooter s, Hood h) {
        this(v, s, h, () -> false);
    }

    public AutoShot(Vision v, Shooter s, Hood h, BooleanSupplier shooting) {
        m_vision = v;
        m_hood = h;
        m_shooter = s;
        m_shooting = shooting;
        
        addRequirements(v);
        addRequirements(s);
        addRequirements(h);
        super.setName("AutoShot");
    }

    public void execute() {
        m_distanceSeen = m_vision.getDistance();
        m_vision.setLimelightOffset();

        double interpolatedAngle = Constants.ShooterConstants.kHoodAngleLookupTable.get(m_distanceSeen);
        double interpolatedRPM = Constants.ShooterConstants.kShooterRPMLookupTable.get(m_distanceSeen);

        if (m_distanceSeen == 0) {
            m_hood.setHoodAngle(Constants.kFenderShotHoodAngle);
            m_shooter.setGoalStatic(Constants.kFenderShotVelocity, false);
        } else {
            m_hood.setHoodAngle(interpolatedAngle);
            m_shooter.setGoalStatic(interpolatedRPM, false);
        }
        SmartDashboard.putNumber("Interpolated RPM", interpolatedRPM);
    }

    public void end(boolean interrupted) {
        m_vision.setLimelightOffset(0);
    }

    public boolean isFinished() {
        return m_shooting.getAsBoolean();

    }

}