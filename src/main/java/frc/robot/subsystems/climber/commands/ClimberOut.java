package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;

public class ClimberOut extends CommandBase {

    private final Climber climber_ = Climber.getInstance();

    public ClimberOut() {
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        climber_.setPivotGoal(-90);
    }

    @Override
    public boolean isFinished() {
        return climber_.m_isAtPivotGoal;
    }
    
}