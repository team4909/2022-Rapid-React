package frc.lib.swervedrivespecialties.swervelib;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState();

    /**
     * Returns the last set desired state of the module. Can me useful for keeping the wheels set to a
     * specific orientation when no demand is given, instead of using some default.
     *
     * @return The last set desired state of the module.
     */
    public SwerveModuleState getDesiredState();

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState);

    /** Reset the drive encoder to zero. */
    public void resetEncoders();
}
