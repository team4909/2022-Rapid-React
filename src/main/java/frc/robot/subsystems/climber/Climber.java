package frc.robot.subsystems.climber;

import java.time.DateTimeException;
import java.util.Map;
import java.util.function.BooleanSupplier;

import javax.swing.text.AbstractDocument.BranchElement;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.AHRSProtocol.IntegrationControl;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.utils.PIDGains;


public class Climber extends SubsystemBase {

    //#region Member Variables
    private static Climber instance_ = null;
    private static final int kTimeoutMs = 100;
    private static final double kVelocityConversion = 600 / 2048d;
    
    private CANSparkMax elevatorLeft_;
    private CANSparkMax elevatorRight_;

    private TalonFX pivotLeft_;
    private TalonFX pivotRight_;

    private SparkMaxPIDController elevatorControllerR_;
    private SparkMaxPIDController elevatorControllerL_;

    private PIDGains m_elevatorUnloadedGains;
    private ElevatorFeedforward m_elevatorFF;

    public boolean haltingPivot_;

    private ClimberStates state_;
    private ClimberStates lastState_;

    private ShuffleboardLayout climberLayout;
    private NetworkTableEntry stateEntry;
    private NetworkTableEntry pivotPos;
    private NetworkTableEntry elevatorPos;
    private NetworkTableEntry pivotVoltage;

    private TrapezoidProfile.State m_targetState;
    private TrapezoidProfile.State m_beginLState;
    private TrapezoidProfile.State m_beginRState;
    private TrapezoidProfile m_profileL;
    private TrapezoidProfile m_profileR;
    
    private Timer m_trapTimer;

    //#endregion

    public enum ClimberStates {
        IDLE("IDLE"), 
        CALIBRATE("CALIBRATE"),
        MID_ALIGN("ALIGNMENT TO MID BAR"),
        MID_CLIMB("CLIMB TO MID"),
        HIGHER_CLIMB("CLIMB HIGHER");

        String state_name;

        private ClimberStates(String name) {
            this.state_name = name;
        }

        public String toString() {
            return this.state_name;
        }
    }

    private Climber() {
        //Elevator: NEOs (CANSparkMaxs)
        //Pivot: Falcons (TalonFXs)
        
        //#region Motor Config
        elevatorLeft_ = new CANSparkMax(Constants.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorRight_ = new CANSparkMax(Constants.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorRight_.setInverted(true);
       // elevatorLeft_.follow(elevatorRight_, true);
        elevatorLeft_.clearFaults();
        elevatorRight_.clearFaults();
        elevatorRight_.getEncoder().setPosition(0);
        elevatorLeft_.getEncoder().setPosition(0);
        elevatorLeft_.setSmartCurrentLimit(80);
        elevatorRight_.setSmartCurrentLimit(80);
        elevatorLeft_.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);
        elevatorRight_.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);
        elevatorLeft_.setClosedLoopRampRate(0.2);
        elevatorRight_.setClosedLoopRampRate(0.2);

        elevatorControllerR_ = elevatorRight_.getPIDController();
        elevatorControllerL_ = elevatorLeft_.getPIDController();
        m_elevatorUnloadedGains = new PIDGains(Constants.ELEVATOR_KP, Constants.ELEVATOR_KI, Constants.ELEVATOR_KD, Constants.ELEVATOR_KF);

        setPIDGains(elevatorControllerR_, m_elevatorUnloadedGains, Constants.Climber.kElevatorPIDSlot);
        setPIDGains(elevatorControllerL_, m_elevatorUnloadedGains, Constants.Climber.kElevatorPIDSlot);
        m_elevatorFF = Constants.Climber.kElevatorFFContraints;

        pivotRight_ = new TalonFX(Constants.RIGHT_PIVOT_MOTOR);
        pivotLeft_ = new TalonFX(Constants.LEFT_PIVOT_MOTOR);
        pivotRight_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 200);
        pivotLeft_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 200);

        pivotRight_.clearStickyFaults(kTimeoutMs);
        // pivotLeft_.follow(pivotRight_);
        pivotLeft_.setInverted(InvertType.InvertMotorOutput);
        pivotRight_.setInverted(InvertType.None);

        pivotLeft_.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        pivotRight_.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        pivotLeft_.setNeutralMode(NeutralMode.Brake);
        pivotRight_.setNeutralMode(NeutralMode.Brake);

        pivotLeft_.config_kP(0, Constants.PIVOT_KP);
        pivotLeft_.config_kI(0, Constants.PIVOT_KI);
        pivotLeft_.config_kD(0, Constants.PIVOT_KD);
        pivotLeft_.config_kF(0, Constants.PIVOT_KF);

        pivotRight_.config_kP(0, Constants.PIVOT_KP);
        pivotRight_.config_kI(0, Constants.PIVOT_KI);
        pivotRight_.config_kD(0, Constants.PIVOT_KD);
        pivotRight_.config_kF(0, Constants.PIVOT_KF);
        pivotRight_.config_IntegralZone(0, (int) (200 / kVelocityConversion));
        pivotLeft_.config_IntegralZone(0, (int) (200 / kVelocityConversion));
        //#endregion

        //#region Shuffleboard Shennaigans
        climberLayout = Shuffleboard.getTab("Driver")
            .getLayout("Climber", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withProperties(Map.of("Label position", "TOP"))
            .withPosition(1,1);

        stateEntry = climberLayout.add("Climber State", "State Not Found").getEntry();
        pivotPos = climberLayout.add("Pivot Position", "Position Not Found").getEntry();
        elevatorPos = climberLayout.add("Elevator Position", "Position Not Found").getEntry();
        pivotVoltage = climberLayout.add("Pivot Voltage", "Voltage Not Found").getEntry();
        //#endregion

        state_ = lastState_ = ClimberStates.IDLE;

    }

    public static Climber getInstance() {
        if (instance_ == null) {
            instance_ = new Climber();
        }

        return instance_;
    }

    private class ClimberCommandBuilder extends CommandBase {
        private final Runnable m_init;
        private final Runnable m_execute;
        private final BooleanSupplier m_isFinished;

        public ClimberCommandBuilder (Runnable execute, BooleanSupplier isFinished, Subsystem... subsystem) {
            this(null, execute, isFinished, subsystem);
        }

        public ClimberCommandBuilder (Runnable init, Runnable execute, BooleanSupplier isFinished, Subsystem... subsystem) {
            m_init = init;
            m_execute = execute;
            m_isFinished = isFinished;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            if (m_init != null) {
                m_init.run();
            }
        }
    
        @Override
        public void execute() {
            m_execute.run();
        }
    
        @Override
        public boolean isFinished() {
            return m_isFinished.getAsBoolean();
        }

        @Override
        public void end(boolean interrupted) {
            state_ = ClimberStates.IDLE;
        }
    }

    public void setState(ClimberStates state) {
        state_ = state;
    }

    public void periodic() {
        SmartDashboard.putNumber("key1", elevatorLeft_.getOutputCurrent());
        SmartDashboard.putNumber("key2", elevatorRight_.getOutputCurrent());
        SmartDashboard.putNumber("posel", elevatorRight_.getEncoder().getPosition());
        SmartDashboard.putNumber("piv pos", pivotRight_.getSelectedSensorPosition());
        SmartDashboard.putBoolean("is pivot hold", haltingPivot_);
        
        stateEntry.setString(state_.toString());
        pivotPos.setDouble(pivotRight_.getSelectedSensorPosition());
        elevatorPos.setDouble(elevatorRight_.getEncoder().getPosition());
        pivotVoltage.setDouble(pivotRight_.getMotorOutputVoltage());

        runRoutine();
    }

    private void setPivotGoal(double goal) {
        if (goal == 0) {
            pivotRight_.set(ControlMode.PercentOutput, goal);
            pivotLeft_.set(ControlMode.PercentOutput, goal);
        } else {
            pivotRight_.set(ControlMode.Position, goal);
            pivotLeft_.set(ControlMode.Position, goal);
        }
    }

    private void setElevatorGoal(double goal) {
        if (goal == 0) {
            elevatorControllerL_.setReference(goal, ControlType.kVoltage);
            elevatorControllerR_.setReference(goal, ControlType.kVoltage);
        } else {
            elevatorControllerR_.setReference(goal, ControlType.kPosition);
            elevatorControllerL_.setReference(goal, ControlType.kPosition);
        }
    }

    private void runRoutine() {
        Command currentClimberCommand = null;
        
        //#region State Machine
        switch (state_) {
            case IDLE:
                if (lastState_ != ClimberStates.IDLE) {
                    currentClimberCommand = idleClimber();
                }
                break;
            case CALIBRATE:
                currentClimberCommand = 
                    new SequentialCommandGroup(calibrateClimber(), pivotBackward())
                        .withTimeout(Constants.Climber.kClimberTimeoutLong);
                break;
            case MID_ALIGN:
                currentClimberCommand = 
                    new SequentialCommandGroup(pivotForward(), extendToMid())
                    .withTimeout(Constants.Climber.kClimberTimeoutLong);
                break;
            case MID_CLIMB:
                // Doesn't currently "hold" the pivot but could
                currentClimberCommand = 
                    new SequentialCommandGroup(retractProfiledClimber())
                    .withTimeout(Constants.Climber.kClimberTimeoutLong);
                break;
            case HIGHER_CLIMB:
                currentClimberCommand = 
                    new SequentialCommandGroup(detach(), pivotBackward(), extendToHigh(),
                                                pivotClimbingHold(), retractProfiledClimber())
                    .withTimeout(Constants.Climber.kClimberTimeoutLong);
                break;
            default:
                state_ = ClimberStates.IDLE;
                break;
        }

        lastState_ = state_;
        currentClimberCommand.schedule();
        //#endregion

    }

    /// PRIVATE COMMANDS ///
    private final Command idleClimber() {
        return new ClimberCommandBuilder(
            () -> { setElevatorGoal(0); 
                    setPivotGoal(0); },
            () -> false,
            this);
    } 

    private final Command calibrateClimber() {
        return new ClimberCommandBuilder(
            () -> { pivotRight_.set(ControlMode.PercentOutput, 0.15); 
                    pivotLeft_.set(ControlMode.PercentOutput, 0.15); },
            () -> pivotRight_.getStatorCurrent() > 15 && pivotLeft_.getStatorCurrent() > 15, 
            this).andThen(new InstantCommand(() -> { pivotLeft_.setSelectedSensorPosition(0); 
                                                     pivotRight_.setSelectedSensorPosition(0); }));
    }

    private final Command pivotForward() {
        return new ClimberCommandBuilder(
            () -> { setPivotGoal(Constants.Climber.kPivotForward); }, 
            () -> true, 
            this);
    }

    private final Command pivotClimbingHold() {
        return new ClimberCommandBuilder(
            () -> { setPivotGoal(Constants.Climber.kMidPivotHold); }, 
            () -> true, 
            this);
    }

    private final Command pivotBackward() {
        return new ClimberCommandBuilder(
            () -> { setPivotGoal(0);}, 
            () -> true, 
            this);
    }

    private final Command extendToMid() {
        return new ClimberCommandBuilder(
            () -> { setElevatorGoal(Constants.Climber.kExtensionMidGoal); }, 
            () -> inTolerance(elevatorRight_.getEncoder().getPosition(), 
                              Constants.Climber.kExtensionMidGoal - 1, 
                              Constants.Climber.kExtensionMidGoal + 1), 
            this);
    }

    private final Command extendToHigh() {
        return new ClimberCommandBuilder(
            () -> { setElevatorGoal(Constants.Climber.kExtensionHighGoal); }, 
            () -> inTolerance(elevatorRight_.getEncoder().getPosition(), 
                              Constants.Climber.kExtensionHighGoal - 1, 
                              Constants.Climber.kExtensionHighGoal + 1), 
            this);
    }

    private final Command detach() {
        return new ClimberCommandBuilder(
            () -> { setElevatorGoal(Constants.Climber.kExtensionDetach); }, 
            () -> inTolerance(elevatorRight_.getEncoder().getPosition(), 
                              Constants.Climber.kExtensionDetach - 1, 
                              Constants.Climber.kExtensionDetach + 1), 
            this);

    }

    // Non-profiled retraction, unused
    private final Command retractClimber() {
        return new ClimberCommandBuilder(
            () -> { setPivotGoal(Constants.Climber.kMidPivotHold);
                    setElevatorGoal(Constants.Climber.kExtensionBottom); }, 
            () -> inTolerance(elevatorRight_.getEncoder().getPosition(), 
                              Constants.Climber.kExtensionBottom - 1, 
                              Constants.Climber.kExtensionBottom + 1), 
            this);
    }

    private Command retractProfiledClimber() {
        return new ClimberCommandBuilder(
            () -> { m_targetState = new TrapezoidProfile.State(Constants.Climber.kExtensionBottom, 0.0);

                    m_beginLState = new TrapezoidProfile.State(elevatorLeft_.getEncoder().getPosition(), 0.0);
                    m_beginRState = new TrapezoidProfile.State(elevatorRight_.getEncoder().getPosition(), 0.0);

                    m_profileL = new TrapezoidProfile(Constants.Climber.kEleavatorTrapConstraints, 
                                                      m_targetState, m_beginLState);
                    m_profileR = new TrapezoidProfile(Constants.Climber.kEleavatorTrapConstraints, 
                                                      m_targetState, m_beginRState);
                    m_trapTimer = new Timer();
        
                    m_trapTimer.reset();
                    m_trapTimer.start();
                },
            () -> { double dt = m_trapTimer.get();
                    // Left Side
                    TrapezoidProfile.State currentLState = m_profileL.calculate(dt);
                    double leftFF = m_elevatorFF.calculate(currentLState.velocity);
                    // Right Side
                    TrapezoidProfile.State currentRState = m_profileR.calculate(dt);
                    double rightFF = m_elevatorFF.calculate(currentRState.velocity);

                    // Set the goals in the controllers
                    elevatorControllerL_.setReference(
                        currentLState.position, ControlType.kPosition, 0, leftFF);
                    elevatorControllerR_.setReference(
                        currentRState.position, ControlType.kPosition, 0, rightFF);
                    },
            () -> true,
            this);
            
        
    }

    private void setPIDGains(SparkMaxPIDController c, PIDGains gains, int slot) {
        c.setP(gains.kP, slot);
        c.setI(gains.kI, slot);
        c.setD(gains.kD, slot);
        c.setFF(gains.kF, slot);
    }

    private boolean inTolerance(double value, double min, double max) {
        if (value > 0) {
            return value < max && value > min;
        } 
 
        return value < min && value > max;
    }

}