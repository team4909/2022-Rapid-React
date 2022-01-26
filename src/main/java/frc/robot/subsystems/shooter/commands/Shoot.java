package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shoot extends ParallelDeadlineGroup {
    public Shoot(){
        super(
            new WaitCommand(2), // feed
            new WaitCommand(2)); // rev
    }
}