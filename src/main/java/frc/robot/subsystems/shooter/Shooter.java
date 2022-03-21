package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Map;

public class Shooter extends SubsystemBase {

    // Object instances
    private static Shooter instance_ = null;
    public static boolean m_shooterDebug = true;

    private final TalonFX flywheel_;
    private final CANSparkMax backSpinWheel_;
    private final Shooter.ShooterDisplay m_shooterDisplay;
    private final SparkMaxPIDController backSpinPID;
    private final SimpleMotorFeedforward m_flywheelFF;
    private final SimpleMotorFeedforward m_backspinFF;

    // Constants
    private final static int kTimeoutMs = 100;
    private static double kFlywheelVelocityConversion = 600.0 / 2048; // native units to rpm
    private final static int kShooterTolerance = 100; //TODO bad tolerance cause bad PID

    // State of the shooter
    private double goalDemand_ = 0.0;
    private static double acceleratorDemand_ = 0.0;
    private boolean runningOpenLoop_ = true;

    private MedianFilter movingFilter_;
    private static double movingAverage_;
    private Timer m_shooterTimer;
    private double m_lastTime;


    private Shooter() {
        flywheel_ = new TalonFX(13);
        backSpinWheel_ = new CANSparkMax(24, MotorType.kBrushless);
        m_flywheelFF = Constants.Shooter.kFlywheelFFConstraints;
        m_backspinFF = Constants.Shooter.kBackspinFFConstraints;

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
        flywheel_.config_kP(0, Constants.Shooter.kFlywheelPIDGains.kP, kTimeoutMs);
        flywheel_.config_kI(0, Constants.Shooter.kFlywheelPIDGains.kI, kTimeoutMs);
        flywheel_.config_kD(0, Constants.Shooter.kFlywheelPIDGains.kD, kTimeoutMs);
        flywheel_.config_kF(0, 0.005);
        flywheel_.config_IntegralZone(0, (int) (200.0 / kFlywheelVelocityConversion));
        flywheel_.selectProfileSlot(0, 0);
        flywheel_.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);

        flywheel_.configClosedloopRamp(0.2);

        flywheel_.set(ControlMode.PercentOutput, 0);
    
        backSpinWheel_.setInverted(true);
        backSpinWheel_.setClosedLoopRampRate(0.2);
        backSpinPID = backSpinWheel_.getPIDController();
        backSpinPID.setP(Constants.Shooter.kBackspinPIDGains.kP, 0);
        backSpinPID.setI(Constants.Shooter.kBackspinPIDGains.kI, 0);
        backSpinPID.setD(Constants.Shooter.kBackspinPIDGains.kD, 0);
        backSpinPID.setFF(0.0001);
        backSpinWheel_.setPeriodicFramePeriod(PeriodicFrame.kStatus0, Constants.kTimeoutMs);


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
    }

    public void setOpenLoopGoal(double goal) {
        goalDemand_ = goal;
        acceleratorDemand_ = goal;
        runningOpenLoop_ = true;
    }

    public boolean spunUp() {
        double currentVelocity = movingAverage_ * kFlywheelVelocityConversion;
        if (goalDemand_ > 0) {
            return Math.abs(goalDemand_ - currentVelocity) < kShooterTolerance;
        }
        return false;
    }

    private void setGoal(double g) {
        this.goalDemand_ = g;
        this.runningOpenLoop_ = false;
        flywheel_.config_kF(0, MathUtil.clamp(0.000002 * goalDemand_, 0.0025, 0.0075));
        // backSpinPID.setFF(MathUtil.clamp(0.0000002 * goalDemand_, 0.00025, 0.00075));
    }

    public InstantCommand setGoalDemand(double goal) {
        return new InstantCommand(() -> {setGoal(goal);});
    }

    public void periodic() {
        if (!runningOpenLoop_) {
            flywheel_.set(ControlMode.Velocity,goalDemand_ / kFlywheelVelocityConversion);
            // flywheel_.set(ControlMode.PercentOutput, bangBangController.calculate(flywheel_.getSelectedSensorVelocity(), goalDemand_) + 0.1);
            // double arbFFValue_b = m_backspinFF.calculate(backSpinWheel_.getEncoder().getVelocity(), goalDemand_ * 8, 0.2);
            // backSpinPID.setReference(goalDemand_ * 4, CANSparkMax.ControlType.kVelocity, 0, arbFFValue_b);
            backSpinPID.setReference(8000, ControlType.kVelocity);
            // backSpinWheel_.set(bangBangControllerBack.calculate(backSpinWheel_.getEncoder().getVelocity(), goalDemand_ * 4));
        } else {
            flywheel_.set(ControlMode.PercentOutput, goalDemand_);
            backSpinPID.setReference(goalDemand_, ControlType.kDutyCycle); 
        }

        movingAverage_ = movingFilter_.calculate(flywheel_.getSelectedSensorVelocity());
        if (m_shooterDebug) m_shooterDisplay.periodic();

        // SmartDashboard.putBoolean("running open loop", runningOpenLoop_);
        // SmartDashboard.putBoolean("Shooter At Speed", spunUp());
        // SmartDashboard.putNumber("Shooter speed", flywheel_.getSelectedSensorVelocity() * kFlywheelVelocityConversion); // ticks * rpm / ticks
        // SmartDashboard.putNumber("BackSpinSHooterSPeed", backSpinWheel_.getEncoder().getVelocity());
    }

    
    // public RunCommand runShooter(double goal) {
    //     m_shooterTimer = new Timer();
    //     m_shooterTimer.reset();
    //     m_shooterTimer.start();
    //     m_lastTime = 0;

    //     return new RunCommand(() -> {
    //         double arbFFValue_f = m_flywheelFF.calculate(flywheel_.getSelectedSensorVelocity(), goal / kFlywheelVelocityConversion, m_shooterTimer.get() - m_lastTime);
    //         flywheel_.set(ControlMode.Velocity, goal / kFlywheelVelocityConversion, DemandType.ArbitraryFeedForward, arbFFValue_f);
    //         double arbFFValue_b = m_backspinFF.calculate(backSpinWheel_.getEncoder().getVelocity(), goal * 8, m_shooterTimer.get() - m_lastTime);
    //         m_lastTime = m_shooterTimer.get();
    //         backSpinPID.setReference(goal * 4, CANSparkMax.ControlType.kVelocity, 0, arbFFValue_b);
    //     }, this);
    // }

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
            m_flywheelSpeed.setDouble(flywheel_.getSelectedSensorVelocity());
            
            if (m_setters.getBoolean(false)) {
                setGoal(m_flywheelSetpointSpeed.getDouble(0));
                // runShooter(m_flywheelSetpointSpeed.getDouble(0)).schedule();
                // flywheel_.set(ControlMode.Velocity, m_flywheelSetpointSpeed.getDouble(0));

                // flywheel_.config_kP(0, m_flywheelP.getDouble(1)); //TODO add p as constant
                // flywheel_.config_kF(0, m_flywheelF.getDouble(0));
                

                // backSpinPID.setReference(m_backSetpointSpeed.getDouble(0), ControlType.kVelocity);
                // backSpinPID.setP(m_backSpinP.getDouble(0), 0); //TODO add p as constant   
                // backSpinPID.setFF(m_backSpinF.getDouble(0), 0);
            }
        }
    }

}
