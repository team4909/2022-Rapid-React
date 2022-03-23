package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShot extends CommandBase {

    private final VisionSubsystem m_vision;
    private final Shooter m_shooter;
    private final Hood m_hood;

    private static double m_distanceSeen = 0;

    public AutoShot(VisionSubsystem v, Shooter s, Hood h) {
        m_vision = v;
        m_hood = h;
        m_shooter = s;

        addRequirements(v);
        addRequirements(s);
        addRequirements(h);
       
    }

    @Override
    public void execute() {
        m_distanceSeen = m_vision.getDistance();

        double interpolatedAngle = Constants.Shooter.kHoodAngleLookupTable.get(m_distanceSeen);
        double interpolatedRPM = Constants.Shooter.kShooterRPMLookupTable.get(m_distanceSeen);
        m_hood.setHoodAngle(interpolatedAngle);
        m_shooter.setGoalStatic(interpolatedRPM);
    }

}