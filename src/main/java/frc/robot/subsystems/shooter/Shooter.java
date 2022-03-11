package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.ArrayList;

public class Shooter extends SubsystemBase {

    // Object instances
    private static Shooter instance_ = null;
    private static boolean hoodDebug_ = true;

//    private final CANSparkMax
    private final TalonFX leader_;
    private final TalonFX follower_;
    private final Solenoid hoodSolenoid_;

    // Constants
    private final static int kTimeoutMs = 100;
    private static double kFlywheelVelocityConversion = 600.0 / 2048.0; // native units to rpm
    private final static int kShooterTolerance = 100; //TODO bad tolerance cause bad PID

    // State of the shooter
    private static double goalDemand_ = 0.0;
    private static double acceleratorDemand_ = 0.0;
    private static boolean runningOpenLoop_ = false;
    private static boolean hoodUp_ = false;

    private MedianFilter movingFilter_;
    private static double movingAverage_;


    private Shooter() {
        leader_ = new TalonFX(13);
        follower_ = new TalonFX(14);
        hoodSolenoid_ = new Solenoid(PneumaticsModuleType.REVPH, 2);

        // General Motor Configuration for the TalonFXs
        leader_.clearStickyFaults(kTimeoutMs);
        leader_.configNominalOutputForward(0, kTimeoutMs);
        leader_.configNominalOutputReverse(0, kTimeoutMs);
        leader_.configNeutralDeadband(0.04, kTimeoutMs);

        leader_.configPeakOutputForward(1.0, kTimeoutMs);
        leader_.configPeakOutputReverse(-1.0, kTimeoutMs);

        leader_.configVoltageCompSaturation(12.0, kTimeoutMs);
        leader_.enableVoltageCompensation(true);
        leader_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 200);
        follower_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 200);

        // Set the follower
        follower_.setInverted(TalonFXInvertType.FollowMaster);
        follower_.follow(leader_);
        // TODO: set inverted 

        // Control Loop Configuration
        leader_.config_kP(0, Constants.kShooterP, kTimeoutMs);
        leader_.config_kI(0, Constants.kShooterI, kTimeoutMs);
        leader_.config_kD(0, Constants.kShooterD, kTimeoutMs);
        leader_.config_kF(0, Constants.kShooterFF, kTimeoutMs);
        leader_.config_IntegralZone(0, (int) (200.0 / kFlywheelVelocityConversion));
        leader_.selectProfileSlot(0, 0);
        leader_.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);

        leader_.configClosedloopRamp(0.2);

        leader_.set(ControlMode.PercentOutput, 0);

        follower_.config_kP(0, Constants.kShooterP, kTimeoutMs);
        follower_.config_kI(0, Constants.kShooterI, kTimeoutMs);
        follower_.config_kD(0, Constants.kShooterD, kTimeoutMs);
        follower_.config_kF(0, Constants.kShooterFF, kTimeoutMs);
        follower_.config_IntegralZone(0, (int) (200.0 / kFlywheelVelocityConversion));
        follower_.selectProfileSlot(0, 0);
        follower_.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);

        follower_.configClosedloopRamp(0.2);

        follower_.set(ControlMode.PercentOutput, 0);
        // follower_.setInverted(true);

        movingFilter_ = new MedianFilter(20);
        movingAverage_ = 0;

    }

    public static Shooter getInstance() {
        if (instance_ == null) {
            instance_ = new Shooter();
        } 
        return instance_;
    }

    public void stop() {
        setOpenLoopGoal(0.0);
        hoodUp_ = false;
    }

    public void setOpenLoopGoal(double goal) {
        goalDemand_ = goal;
        acceleratorDemand_ = goal;
        runningOpenLoop_ = true;
    }

    public void setDiffVelocityGoal(double goal, double accelerator, boolean hoodUp) {
        goalDemand_ = goal;
        acceleratorDemand_ = accelerator;
        runningOpenLoop_ = false;
        hoodUp_ = hoodUp;
    }

    public void setVelocityGoal(double goal, boolean hoodUp) {
        goalDemand_ = goal;
        acceleratorDemand_ = goal;
        runningOpenLoop_ = false;
        hoodUp_ = hoodUp;
    }

    public boolean spunUp() {
        double currentVelocity = movingAverage_ * kFlywheelVelocityConversion;
        if (goalDemand_ > 0) {
            return Math.abs(goalDemand_ - currentVelocity) < kShooterTolerance;
        }
        return false;
    }

    @Override
    public void periodic() {
        hoodSolenoid_.set(hoodUp_);
        movingAverage_ = movingFilter_.calculate(leader_.getSelectedSensorVelocity());
        if (!runningOpenLoop_) {
            leader_.set(ControlMode.Velocity, goalDemand_ / kFlywheelVelocityConversion);
            follower_.set(ControlMode.Velocity, acceleratorDemand_ / kFlywheelVelocityConversion);

        } else {
            leader_.set(ControlMode.PercentOutput, goalDemand_);
            follower_.set(ControlMode.PercentOutput, acceleratorDemand_);
        }
        SmartDashboard.putBoolean("Shooter At Speed", spunUp());
        SmartDashboard.putNumber("Shooter speed", leader_.getSelectedSensorVelocity() * kFlywheelVelocityConversion);
    }

    private class ShooterDisplay {
        private ShuffleboardTab m_tab = Shuffleboard.getTab("Debug");
        private ShuffleboardLayout m_layout = m_tab.getLayout("Hood", BuiltInLayouts.kList);
        private NetworkTableEntry m_shooterSpeed;
    }

}
