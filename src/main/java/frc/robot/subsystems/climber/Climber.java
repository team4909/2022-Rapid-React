package frc.robot.subsystems.climber;

import java.util.Map;
import java.util.function.BooleanSupplier;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.PIDGains;


public class Climber extends SubsystemBase {

    //#region Member Variables
    private static Climber m_instance = null;
    
    
    private CANSparkMax m_leftElevatorMotor;
    private CANSparkMax m_rightElevatorMotor;

    private TalonFX m_leftPivot;
    private TalonFX m_rightPivot;

    private SparkMaxPIDController m_rightElevatorController;
    private SparkMaxPIDController m_leftElevatorController;

    private PIDGains m_elevatorUnloadedGains;
    private ElevatorFeedforward m_elevatorFF;


    private ClimberStates m_state;
    private ClimberStates m_lastState;

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
        m_leftElevatorMotor = new CANSparkMax(Constants.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
        m_rightElevatorMotor = new CANSparkMax(Constants.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);
        m_leftElevatorMotor.setInverted(true);
        m_rightElevatorMotor.setInverted(false);
       // elevatorLeft_.follow(elevatorRight_, true);
        m_leftElevatorMotor.clearFaults();
        m_rightElevatorMotor.clearFaults();
        m_rightElevatorMotor.getEncoder().setPosition(0);
        m_leftElevatorMotor.getEncoder().setPosition(0);
        m_leftElevatorMotor.setSmartCurrentLimit(80);
        m_rightElevatorMotor.setSmartCurrentLimit(80);
        m_leftElevatorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, Constants.kTimeoutMs);
        m_rightElevatorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, Constants.kTimeoutMs);
        m_leftElevatorMotor.setClosedLoopRampRate(0.2);
        m_rightElevatorMotor.setClosedLoopRampRate(0.2);

        m_rightElevatorController = m_rightElevatorMotor.getPIDController();
        m_leftElevatorController = m_leftElevatorMotor.getPIDController();
        m_elevatorUnloadedGains = new PIDGains(Constants.ELEVATOR_KP, Constants.ELEVATOR_KI, Constants.ELEVATOR_KD, Constants.ELEVATOR_KF);

        setPIDGains(m_rightElevatorController, m_elevatorUnloadedGains, Constants.Climber.kElevatorPIDSlot);
        setPIDGains(m_leftElevatorController, m_elevatorUnloadedGains, Constants.Climber.kElevatorPIDSlot);
        m_elevatorFF = Constants.Climber.kElevatorFFContraints;

        m_rightPivot = new TalonFX(Constants.RIGHT_PIVOT_MOTOR);
        m_leftPivot = new TalonFX(Constants.LEFT_PIVOT_MOTOR);
        m_rightPivot.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.kTimeoutMs);
        m_leftPivot.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.kTimeoutMs);

        m_rightPivot.clearStickyFaults(Constants.kTimeoutMs);
        // pivotLeft_.follow(pivotRight_);
        m_leftPivot.setInverted(InvertType.InvertMotorOutput);
        m_rightPivot.setInverted(InvertType.None);

        m_leftPivot.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_rightPivot.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_leftPivot.setNeutralMode(NeutralMode.Brake);
        m_rightPivot.setNeutralMode(NeutralMode.Brake);

        m_leftPivot.config_kP(0, Constants.PIVOT_KP);
        m_leftPivot.config_kI(0, Constants.PIVOT_KI);
        m_leftPivot.config_kD(0, Constants.PIVOT_KD);
        m_leftPivot.config_kF(0, Constants.PIVOT_KF);

        m_rightPivot.config_kP(0, Constants.PIVOT_KP);
        m_rightPivot.config_kI(0, Constants.PIVOT_KI);
        m_rightPivot.config_kD(0, Constants.PIVOT_KD);
        m_rightPivot.config_kF(0, Constants.PIVOT_KF);
        m_rightPivot.config_IntegralZone(0, (int) (200 / Constants.Climber.kClimberVelocityConversion));
        m_leftPivot.config_IntegralZone(0, (int) (200 / Constants.Climber.kClimberVelocityConversion));
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

        m_state = m_lastState = ClimberStates.IDLE;

    }

    public static Climber getInstance() {
        if (m_instance == null) {
            m_instance = new Climber();
        }

        return m_instance;
    }

    private class ClimberCommandBuilder extends CommandBase {
        private final Runnable m_init;
        private final Runnable m_execute;
        private final BooleanSupplier m_isFinished;
        private final ClimberStates m_transitionState;

        public ClimberCommandBuilder (Runnable execute, BooleanSupplier isFinished, Subsystem... subsystem) {
            this(null, execute, isFinished, null, subsystem);
        }
        
        public ClimberCommandBuilder (Runnable execute, BooleanSupplier isFinished, ClimberStates transition, Subsystem... subsystem) {
            this(null, execute, isFinished, transition, subsystem);
        }

        public ClimberCommandBuilder (Runnable init, Runnable execute, BooleanSupplier isFinished, ClimberStates transition, Subsystem... subsystem) {
            m_init = init;
            m_execute = execute;
            m_isFinished = isFinished;
            m_transitionState = transition;
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
            if (m_transitionState != null)
                m_state = m_transitionState;
        }
    }

    public void setState(ClimberStates state) {
        m_state = state;
    }

    public void periodic() {
        SmartDashboard.putBoolean("key1", inTolerance(m_leftPivot.getSelectedSensorPosition(), 
                                            Constants.Climber.kPivotForward + 200, 
                                            Constants.Climber.kPivotForward - 200));
        SmartDashboard.putNumber("key2", m_leftPivot.getStatorCurrent());
        SmartDashboard.putNumber("posel", m_rightElevatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("piv pos", m_leftPivot.getSelectedSensorPosition());
        SmartDashboard.putNumber("piv vol", m_rightPivot.getMotorOutputVoltage());
        SmartDashboard.putString("Climber State", m_state.name());

        stateEntry.setString(m_state.toString());
        pivotPos.setDouble(m_rightPivot.getSelectedSensorPosition());
        elevatorPos.setDouble(m_rightElevatorMotor.getEncoder().getPosition());
        pivotVoltage.setDouble(m_rightPivot.getMotorOutputVoltage());

        runRoutine();
    }

    private void setPivotGoal(double goal) {
        if (goal == 0) {
            m_rightPivot.set(TalonFXControlMode.PercentOutput, goal);
            m_leftPivot.set(TalonFXControlMode.PercentOutput, goal);
        } else {
            m_rightPivot.set(TalonFXControlMode.Position, goal);
            m_leftPivot.set(TalonFXControlMode.Position, goal);
        }
    }

    private void setElevatorGoal(double goal) {
        if (goal == 0) {
            m_leftElevatorController.setReference(goal, ControlType.kVoltage);
            m_rightElevatorController.setReference(goal, ControlType.kVoltage);
        } else {
            m_rightElevatorController.setReference(goal, ControlType.kPosition);
            m_leftElevatorController.setReference(goal, ControlType.kPosition);
        }
    }

    private void runRoutine() {
        Command currentClimberCommand = null;
        
        //#region State Machine
        if (m_state != m_lastState){
            switch (m_state) {
                case IDLE:
                    currentClimberCommand = idleClimber();
                    
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
                    m_state = ClimberStates.IDLE;
                    break;
            }
        }

        m_lastState = m_state;
        if (currentClimberCommand != null)
            currentClimberCommand.schedule();
        //#endregion

    }

    /// PRIVATE COMMANDS ///
   private final Command setDefaultState(ClimberStates state) {
       return new InstantCommand(() -> m_state = state);
   }

    private final Command idleClimber() {
        return new ClimberCommandBuilder(
            () -> { setElevatorGoal(0); 
                    setPivotGoal(0); },
            () -> true,
            this);
    } 

    private final Command calibrateClimber() {
        return new ClimberCommandBuilder(
            () -> { m_rightPivot.set(ControlMode.PercentOutput, 0.15); 
                    m_leftPivot.set(ControlMode.PercentOutput, 0.15); },
            () -> m_leftPivot.getStatorCurrent() > 30 && m_rightPivot.getStatorCurrent() > 30, 
            ClimberStates.IDLE,
            this).andThen(new InstantCommand(() -> { m_leftPivot.setSelectedSensorPosition(0); 
                                                     m_rightPivot.setSelectedSensorPosition(0); }));
    }

    private final Command pivotForward() {
        return new ClimberCommandBuilder(
            () -> { setPivotGoal(Constants.Climber.kPivotForward); }, 
            () -> pivotTolerance(Constants.Climber.kPivotForward), 
            this);
    }

    private final Command pivotClimbingHold() {
        return new ClimberCommandBuilder(
            () -> { setPivotGoal(Constants.Climber.kMidPivotHold); }, 
            () -> pivotTolerance(Constants.Climber.kMidPivotHold), 
            this);
    }

    private final Command pivotBackward() {
        System.out.println("pivoting backwords \n\n\n\n\n");
        return new ClimberCommandBuilder(
            () -> { setPivotGoal(-500);}, 
            () -> pivotTolerance(-500), 
            this);
    }

    private final Command extendToMid() {
        return new ClimberCommandBuilder(
            () -> { setElevatorGoal(Constants.Climber.kExtensionMidGoal); }, 
            () -> inTolerance(m_rightElevatorMotor.getEncoder().getPosition(), 
                              Constants.Climber.kExtensionMidGoal + 1, 
                              Constants.Climber.kExtensionMidGoal - 1), 
            this);
    }

    private final Command extendToHigh() {
        return new ClimberCommandBuilder(
            () -> { setElevatorGoal(Constants.Climber.kExtensionHighGoal); }, 
            () -> inTolerance(m_rightElevatorMotor.getEncoder().getPosition(), 
                              Constants.Climber.kExtensionHighGoal + 1, 
                              Constants.Climber.kExtensionHighGoal - 1), 
            this);
    }

    private final Command detach() {
        return new ClimberCommandBuilder(
            () -> { setElevatorGoal(Constants.Climber.kExtensionDetach); }, 
            () -> inTolerance(m_rightElevatorMotor.getEncoder().getPosition(), 
                              Constants.Climber.kExtensionDetach + 1, 
                              Constants.Climber.kExtensionDetach - 1), 
            this);

    }

    // Non-profiled retraction, unused
    private final Command retractClimber() {
        return new ClimberCommandBuilder(
            () -> { setPivotGoal(Constants.Climber.kMidPivotHold);
                    setElevatorGoal(Constants.Climber.kExtensionBottom); }, 
            () -> inTolerance(m_rightElevatorMotor.getEncoder().getPosition(), 
                              Constants.Climber.kExtensionBottom - 1, 
                              Constants.Climber.kExtensionBottom + 1), 
            this);
    }

    private Command retractProfiledClimber() {
        return new ClimberCommandBuilder(
            () -> { m_targetState = new TrapezoidProfile.State(Constants.Climber.kExtensionBottom, 0.0);

                    m_beginLState = new TrapezoidProfile.State(m_leftElevatorMotor.getEncoder().getPosition(), 0.0);
                    m_beginRState = new TrapezoidProfile.State(m_rightElevatorMotor.getEncoder().getPosition(), 0.0);

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
                    m_leftElevatorController.setReference(
                        currentLState.position, ControlType.kPosition, 0, leftFF);
                    m_rightElevatorController.setReference(
                        currentRState.position, ControlType.kPosition, 0, rightFF);
                    },
            () -> false,
            null,
            this);
            
        
    }

    private void setPIDGains(SparkMaxPIDController c, PIDGains gains, int slot) {
        c.setP(gains.kP, slot);
        c.setI(gains.kI, slot);
        c.setD(gains.kD, slot);
        c.setFF(gains.kF, slot);
    }

    private boolean pivotTolerance(double t) {
        return inTolerance(m_leftPivot.getSelectedSensorPosition(), 
                              t + 200, 
                              t - 200);
    }

    private boolean elevatorTolerance(double t) {
        return inTolerance(m_rightElevatorMotor.getEncoder().getPosition(), 
                            t + 1, 
                            t - 1);
    }

        // -3724, -3800-400, -3800+400
    private boolean inTolerance(double value, double min, double max) {
        if (value > 0) {
            return value < max && value > min;
        } 
 
        return value < min && value > max;
    }
    private class ClimberDisplay {
        
    }
}