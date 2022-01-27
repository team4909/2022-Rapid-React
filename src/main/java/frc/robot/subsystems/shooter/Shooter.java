package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // Object instances
    private static Shooter instance_;

    private final TalonFX leader_;
    private final TalonFX follower_;

    // Constants
    private final static int kTimeoutMs = 100;
    private static double kFlywheelVelocityConversion = 600.0 / 2048.0; // native units to rpm
    private final static int kShooterTolerance = 100;

    // State of the shooter
    private static double goalDemand_ = 0.0;
    private static boolean runningOpenLoop_ = false;

    private Shooter() {
        leader_ = new TalonFX(10);
        follower_ = new TalonFX(11);

        // General Motor Configuration for the TalonFXs
        leader_.clearStickyFaults(kTimeoutMs);
        leader_.configNominalOutputForward(0, kTimeoutMs);
        leader_.configNominalOutputReverse(0, kTimeoutMs);
        leader_.configNeutralDeadband(0.04, kTimeoutMs);

        leader_.configPeakOutputForward(1.0, kTimeoutMs);
        leader_.configPeakOutputReverse(-1.0, kTimeoutMs);

        leader_.configVoltageCompSaturation(12.0, kTimeoutMs);
        leader_.enableVoltageCompensation(true);

        // Set the follower
        follower_.set(ControlMode.Follower, 11);
        // TODO: set inverted 

        // Control Loop Configuration
        leader_.config_kP(0, 0, kTimeoutMs);
        leader_.config_kI(0, 0, kTimeoutMs);
        leader_.config_kD(0, 0, kTimeoutMs);
        leader_.config_kF(0, 0, kTimeoutMs);
        leader_.config_IntegralZone(0, (int) (200.0 / kFlywheelVelocityConversion));
        leader_.selectProfileSlot(0, 0);
        leader_.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);

        leader_.configClosedloopRamp(0.2);

        leader_.set(ControlMode.PercentOutput, 0);

    }

    public static Shooter getInstance() {
        if (instance_ == null) {
            instance_ = new Shooter();
        } 
        return instance_;
    }

    public void stop() {
        setOpenLoopGoal(0.0);
    }

    public void setOpenLoopGoal(double goal) {
        goalDemand_ = goal;
        runningOpenLoop_ = true;
    }

    public void setVelocityGoal(double goal) {
        goalDemand_ = goal;
        runningOpenLoop_ = false;
    }

    public boolean spunUp() {
        double currentVelocity = leader_.getSelectedSensorVelocity() * kFlywheelVelocityConversion;
        if (goalDemand_ > 0) {
            return Math.abs(goalDemand_ - currentVelocity) < kShooterTolerance;
        }
        return false;
    }


    public void update() {
        if (!runningOpenLoop_) {
            leader_.set(ControlMode.Velocity, goalDemand_ / kFlywheelVelocityConversion);
        } else {
            leader_.set(ControlMode.PercentOutput, goalDemand_);
        }
    }


}
