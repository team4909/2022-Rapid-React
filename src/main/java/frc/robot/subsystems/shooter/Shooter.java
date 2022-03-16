package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Map;

public class Shooter extends SubsystemBase {

    // Object instances
    private static Shooter instance_ = null;
    public static boolean m_shooterDebug = true;

//    private final CANSparkMax
    private final TalonFX flywheel_;
    private final CANSparkMax backSpinWheel_;
    private final Shooter.ShooterDisplay m_shooterDisplay;
    private final SparkMaxPIDController backSpinPID;

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
        flywheel_ = new TalonFX(13);
        backSpinWheel_ = new CANSparkMax(24, MotorType.kBrushless);


        // General Motor Configuration for the TalonFXs
        flywheel_.clearStickyFaults(kTimeoutMs);
        flywheel_.configNominalOutputForward(0, kTimeoutMs);
        flywheel_.configNominalOutputReverse(0, kTimeoutMs);
        flywheel_.configNeutralDeadband(0.04, kTimeoutMs);

        flywheel_.configPeakOutputForward(1.0, kTimeoutMs);
        flywheel_.configPeakOutputReverse(-1.0, kTimeoutMs);

        flywheel_.configVoltageCompSaturation(12.0, kTimeoutMs);
        flywheel_.enableVoltageCompensation(true);
        flywheel_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 200);

        // Control Loop Configuration
        flywheel_.config_kP(0, Constants.kShooterP, kTimeoutMs);
        flywheel_.config_kI(0, Constants.kShooterI, kTimeoutMs);
        flywheel_.config_kD(0, Constants.kShooterD, kTimeoutMs);
        flywheel_.config_kF(0, Constants.kShooterFF, kTimeoutMs);
        flywheel_.config_IntegralZone(0, (int) (200.0 / kFlywheelVelocityConversion));
        flywheel_.selectProfileSlot(0, 0);
        flywheel_.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);

        flywheel_.configClosedloopRamp(0.2);

        flywheel_.set(ControlMode.PercentOutput, 0);
    
        backSpinWheel_.setInverted(true);
        backSpinWheel_.setClosedLoopRampRate(0.2);
        backSpinPID = backSpinWheel_.getPIDController();
        backSpinPID.setP(0.0001);
        backSpinWheel_.setSmartCurrentLimit(30); //TODO idk if we need one but the sparks keep frying
        

        m_shooterDisplay = new Shooter.ShooterDisplay();
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
        movingAverage_ = movingFilter_.calculate(flywheel_.getSelectedSensorVelocity());
        if (!runningOpenLoop_) {
            flywheel_.set(ControlMode.Velocity, goalDemand_ / kFlywheelVelocityConversion);
            backSpinPID.setReference(acceleratorDemand_ / kFlywheelVelocityConversion , ControlType.kVelocity);
        } else {
            flywheel_.set(ControlMode.PercentOutput, goalDemand_);
            backSpinPID.setReference(acceleratorDemand_, ControlType.kVelocity);
        }

        if (m_shooterDebug) m_shooterDisplay.periodic();

        SmartDashboard.putBoolean("Shooter At Speed", spunUp());
        SmartDashboard.putNumber("Shooter speed", flywheel_.getSelectedSensorVelocity() * kFlywheelVelocityConversion);
        SmartDashboard.putNumber("BackSpinSHooterSPeed", backSpinWheel_.getEncoder().getVelocity());
    }

    private class ShooterDisplay {
        private ShuffleboardTab m_tab = Shuffleboard.getTab("Debug");
        private ShuffleboardLayout m_layout = m_tab.getLayout("Shooter", BuiltInLayouts.kGrid)
        .withPosition(2, 0).withSize(6, 6);
        private NetworkTableEntry m_backSpeed, m_backSetpointSpeed, m_backSpinP, m_backSpinF,
                                  m_flywheelSpeed, m_flywheelSetpointSpeed, m_flywheelP, m_flywheelF,
                                  m_setters;

        public ShooterDisplay() {
            m_backSpeed = m_layout.add("Backspin speed", 0).withWidget(BuiltInWidgets.kDial).getEntry();
            m_backSetpointSpeed = m_layout.add("Backspin Setpoint speed", 0d).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("Min", 0, "Max", 6000)).getEntry();
            m_backSpinP = m_layout.addPersistent("P backspin", 1d).withWidget(BuiltInWidgets.kTextView).getEntry();
            m_backSpinF = m_layout.addPersistent("FF backspin", 0d).withWidget(BuiltInWidgets.kTextView).getEntry();
            m_flywheelSpeed = m_layout.add("Flywheel speed", 0).withWidget(BuiltInWidgets.kDial).getEntry();
            m_flywheelSetpointSpeed = m_layout.add("Flywheel Setpoint speed", 0d).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("Min", 0, "Max", 6000)).getEntry();
            
                m_flywheelP = m_layout.addPersistent("P flywheel", Constants.kShooterP).withWidget(BuiltInWidgets.kTextView).getEntry(); 
            m_flywheelF = m_layout.addPersistent("FF flywheel", Constants.kShooterFF).withWidget(BuiltInWidgets.kTextView).getEntry();
            m_setters = m_layout.add("Use Debug Values", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        }

        public void periodic() {
            m_backSpeed.setDouble(backSpinWheel_.getEncoder().getVelocity());
            m_flywheelSpeed.setDouble(flywheel_.getSelectedSensorVelocity() * kFlywheelVelocityConversion);
            
            if (m_setters.getBoolean(false)) {
                flywheel_.set(ControlMode.Velocity, m_flywheelSetpointSpeed.getDouble(0));
                flywheel_.config_kP(0, m_flywheelP.getDouble(1)); //TODO add p as constant
                flywheel_.config_kF(0, m_flywheelF.getDouble(0));
                

                backSpinPID.setReference(m_backSetpointSpeed.getDouble(0), ControlType.kVelocity);
                backSpinPID.setP(m_backSpinP.getDouble(0), 0); //TODO add p as constant   
                backSpinPID.setFF(m_backSpinF.getDouble(0), 0);
            }
        }
    }

}
