package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeFeeder;

public class ReverseIntakeCmd extends CommandBase {
    private final IntakeFeeder intakeFeeder_;

    public ReverseIntakeCmd() {
        intakeFeeder_ = IntakeFeeder.getInstance();
    }

    @Override
    public void initialize() {
        intakeFeeder_.reverseIntake();

    }

    @Override
    public void execute() {
        // Let state machine run
        // Shouldn't need to continuously set the state
    }

    @Override
    public void end(boolean interrupted) {
        intakeFeeder_.stopIntake();

    }
}
