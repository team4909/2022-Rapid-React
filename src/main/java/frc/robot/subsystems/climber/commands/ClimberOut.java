package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class ClimberOut extends CommandBase {

    private double angle_;
    private double goal_;
    private final Climber climber_ = Climber.getInstance();

    public ClimberOut(double angle) {
        angle_ = angle;
    }

    @Override
    public void initialize() {
        goal_ = angle_;
    }

    public void execute() {
        climber_.setPivotGoal(goal_);
    }

    @Override
    public boolean isFinished() {
        return climber_.m_isAtPivotGoal;
    }
    
}