package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberStates;

public class AutoClimb extends SequentialCommandGroup {

    private Climber m_climber = Climber.getInstance();

    public AutoClimb() {
        addCommands(
            new InstantCommand(() -> m_climber.setState(ClimberStates.RETRACTION)),
            new WaitCommand(2),
            new InstantCommand(() -> m_climber.setState(ClimberStates.PREPARE_HIGH)),
            new WaitCommand(3),
            new InstantCommand(() -> m_climber.setState(ClimberStates.HIGHER_CLIMB)),
            new WaitCommand(3),
            new InstantCommand(() -> m_climber.setState(ClimberStates.RETRACTION)),
            new WaitCommand(7),
            new InstantCommand(() -> m_climber.setState(ClimberStates.PREPARE_HIGH)),
            new WaitCommand(3),
            new InstantCommand(() -> m_climber.setState(ClimberStates.HIGHER_CLIMB)),
            new WaitCommand(2),
            new InstantCommand(() -> m_climber.setState(ClimberStates.RETRACTION))
        );

        SmartDashboard.putData(this);
    }

    
}
