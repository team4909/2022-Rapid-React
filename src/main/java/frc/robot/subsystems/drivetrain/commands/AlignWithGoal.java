package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;


import java.util.function.DoubleSupplier;

public class AlignWithGoal extends CommandBase {

    // DrivetrainSubsystem requirment, gets passed in
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    // Movement suppliers for the creation of chassis speeds
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private double offset;
    private double angularSpeed;
    private double angularVelocity;
    private final VisionSubsystem m_visionSubsystem = VisionSubsystem.getInstance();
    private double lastError = m_visionSubsystem.getXDegrees();

    /**
     * 
     * @param drivetrainSubsystem
     *  Drivetrain that this command will drive
     * @param d
     *  DoubleSupplier for the X translantion of the robot
     * @param e
     *  DoubleSupplier for the Y translation of the robot
     * @param rotationSupplier
     *  DoubleSupplier for the Rotation of the robot
     */
    public 
    AlignWithGoal(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier d,
                               DoubleSupplier e,
                               DoubleSupplier rotationSupplier) {


        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = d;
        this.m_translationYSupplier = e;
        this.m_rotationSupplier = rotationSupplier;

        // Adds a DrivetrainSubsystem requirment for this command, makes sure that it can't be called wihtout a drivetrain
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        offset = -m_visionSubsystem.getXDegrees();
        angularSpeed = (offset * Constants.GOAL_ALIGN_KP + Math.abs(offset - lastError) * Constants.GOAL_ALIGN_KD);

        if (SmartDashboard.getBoolean("Align", false) == true){

            if (m_visionSubsystem.isAligned == true) {
                angularVelocity = 0;
            } else {
                if (m_visionSubsystem.getXDegrees() <= 0)
                    angularVelocity = angularSpeed;

                if (m_visionSubsystem.getXDegrees() >= 0) 
                    angularVelocity = angularSpeed;
            }
            // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
            m_drivetrainSubsystem.drive(
                    // fromFielRelativeSpeeds, provides Field Relative drive
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            m_rotationSupplier.getAsDouble() + angularVelocity, //not sure if we need this negative, may be neccesary otherwise it goes backwards?
                            m_drivetrainSubsystem.getGyroscopeRotation()
                    )
            );

            lastError = m_visionSubsystem.getXDegrees();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // When command is scheduled to end, stops moving the drivetrain
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
