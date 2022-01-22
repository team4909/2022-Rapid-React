package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    // DrivetrainSubsystem requirment, gets passed in
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    // Movement suppliers for the creation of chassis speeds
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    /**
     * 
     * @param drivetrainSubsystem
     *  Drivetrain that this command will drive
     * @param translationXSupplier
     *  DoubleSupplier for the X translantion of the robot
     * @param translationYSupplier
     *  DoubleSupplier for the Y translation of the robot
     * @param rotationSupplier
     *  DoubleSupplier for the Rotation of the robot
     */
    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {

        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        // Adds a DrivetrainSubsystem requirment for this command, makes sure that it can't be called wihtout a drivetrain
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                // fromFielRelativeSpeeds, provides Field Relative drive
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        // When command is scheduled to end, stops moving the drivetrain
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
