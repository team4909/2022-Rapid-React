package frc.robot.subsystems.climber;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.commands.Elevator;
import frc.robot.subsystems.climber.commands.Pivot;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
public class Climber extends SubsystemBase {

    //#region Member Variables
    private static Climber instance_ = null;
    private static final int kTimeoutMs = 100;
    private static final double kVelocityConversion = 600 / 2048d;
    
    private CANSparkMax elevatorLeft_;
    private CANSparkMax elevatorRight_;

    private TalonFX pivotLeft_;
    private TalonFX pivotRight_;

    private SparkMaxPIDController elevatorController_;

    public boolean m_isAtElevatorGoal;

    private ClimberStates state_;

    private VoltageTracker voltageTracker_;

    private BooleanSupplier isClimberOut_;    
    private BooleanSupplier shouldRun_;

    private ShuffleboardTab tab = Shuffleboard.getTab("Driver");
    private ShuffleboardLayout climberLayout;
    private NetworkTableEntry stateEntry;
    private NetworkTableEntry pivotPos;
    private NetworkTableEntry elevatorPos;

    //#endregion

    private enum ClimberStates {
        IDLE("IDLE"), 
        PIVOT_FORWARD("PIVOT_FORWARD"), 
        EXTEND_HOOK("EXTEND_HOOK"), 
        BACK_PIVOT("BACK_PIVOT"), 
        RETRACT("RETRACT"), 
        STABILIZE("STABILIZE");

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
        elevatorRight_ = new CANSparkMax(Constants.RIGHT_ELEVATOR_MOTOR, null);
        elevatorLeft_ = new CANSparkMax(Constants.LEFT_ELEVATOR_MOTOR, null);
        elevatorLeft_.follow(elevatorRight_, true);
        elevatorLeft_.clearFaults();
        elevatorRight_.getEncoder().setPosition(0);

        elevatorController_ = elevatorLeft_.getPIDController();
        elevatorController_.setP(Constants.ELEVATOR_KP);
        elevatorController_.setD(Constants.ELEVATOR_KI);
        elevatorController_.setI(Constants.ELEVATOR_KD);
        elevatorController_.setFF(Constants.ELEVATOR_KF);
        

        pivotRight_ = new TalonFX(Constants.RIGHT_ELEVATOR_MOTOR);
        pivotLeft_ = new TalonFX(Constants.LEFT_ELEVATOR_MOTOR);

        pivotRight_.clearStickyFaults(kTimeoutMs);
        pivotLeft_.set(ControlMode.Follower, Constants.RIGHT_PIVOT_MOTOR);
        pivotLeft_.setInverted(InvertType.OpposeMaster);

        pivotLeft_.config_kP(0, Constants.PIVOT_KP);
        pivotLeft_.config_kI(0, Constants.PIVOT_KI);
        pivotLeft_.config_kD(0, Constants.PIVOT_KD);
        pivotLeft_.config_kF(0, Constants.PIVOT_KF);
        pivotLeft_.config_IntegralZone(0, (int) (200 / kVelocityConversion));
        //#endregion

        //#region Shuffleboard Shennaigans
        climberLayout = Shuffleboard.getTab("Driver")
        .getLayout("Climber", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withProperties(Map.of("Label position", "TOP"))
        .withPosition(1,1);

        stateEntry = climberLayout.addPersistent("Climber State", "State Not Found").getEntry();
        pivotPos = climberLayout.addPersistent("Pivot Position", "Position Not Found").getEntry();
        elevatorPos = climberLayout.addPersistent("Elevator Position", "Position Not Found").getEntry();
        //#endregion

        isClimberOut_ = () -> false;
        shouldRun_ = () -> true;

        voltageTracker_ = new VoltageTracker(10);
        state_ = ClimberStates.IDLE;

    }

    public void periodic() {
        stateEntry.setString(state_.toString());
        pivotPos.setDouble(pivotRight_.getSelectedSensorPosition());
        elevatorPos.setDouble(elevatorRight_.getEncoder().getPosition());
    }

    //#region Class Methods
    private double getPivotDegrees() {
        return elevatorRight_.getEncoder().getPosition() / Constants.TICKS_PER_ELEVATOR_INCH;
    }
    
    private double getElevatorInches() {
        return pivotRight_.getSelectedSensorPosition() / Constants.TICKS_PER_ELEVATOR_INCH;
    }

    private boolean inTolerance(double value, double min, double max) {
        return value < max && value > min;
    }

    public void setPivotGoal(double goal) {
        pivotRight_.set(ControlMode.Position, goal * Constants.TICKS_PER_PIVOT_DEGREE);
    }

    public void setElevatorGoal(double goal) {
        elevatorController_.setReference(goal * Constants.TICKS_PER_ELEVATOR_INCH, ControlType.kPosition);
    }

    public void setState(ClimberStates state) {
        state_ = state;
    }

    /**
     * @param out true will extend outwards (up), false will extend inwards (down)
     */
    private void climberDeploy(boolean out) {
        pivotRight_.set(ControlMode.PercentOutput, out ? 0.3 : -0.3); //Deploy at 30%
        double avgV = voltageTracker_.calculate(pivotRight_.getMotorOutputVoltage());
        isClimberOut_ = () -> pivotRight_.getMotorOutputVoltage() > (avgV * 2);
        if (isClimberOut_.getAsBoolean()) {
            voltageTracker_.reset();
            pivotRight_.setSelectedSensorPosition(0); // Zero the pivot motors
        }
        
    }

    private void runRoutine() {
        CommandBase nextPivotCommand = null;
        CommandBase nextElevatorCommand = null;
        
        //#region State Machine
        switch (state_) {
            case IDLE:
                nextPivotCommand = new Pivot(0);
                nextElevatorCommand = new Elevator(0);
                break;
            case PIVOT_FORWARD:
                nextPivotCommand = new Pivot(Constants.BAR_THETA + 10); // The theta is offset by +10 deg so we overshoot the bar
                nextElevatorCommand = new Elevator(0);
                if (this.inTolerance(this.getPivotDegrees(), Constants.BAR_THETA + 2, Constants.BAR_THETA + 2)) { // angle tolerance of 2 deg
                    state_ = ClimberStates.EXTEND_HOOK;
                }
                break;
            case EXTEND_HOOK:
                nextPivotCommand = new Pivot(0);
                nextElevatorCommand = new Elevator(Constants.MAX_ELEVATOR_HEIGHT);
                if (this.inTolerance(this.getElevatorInches(), Constants.MAX_ELEVATOR_HEIGHT - 0.5, Constants.MAX_ELEVATOR_HEIGHT + 0.5)) { // extension tolerance of 1/2 in
                    state_ = ClimberStates.BACK_PIVOT;
                }
                break;
            case BACK_PIVOT:
                nextElevatorCommand = new Elevator(0);
                nextPivotCommand = new Pivot(-Constants.BAR_THETA);
                double avgV = voltageTracker_.calculate(pivotRight_.getMotorOutputVoltage());
                if (pivotRight_.getMotorOutputVoltage() > (avgV * 2)) { //If the current voltage is equal to double the average before it
                        voltageTracker_.reset();
                        state_ = ClimberStates.RETRACT;
                }
                break;
            case RETRACT:
                nextElevatorCommand = new Elevator(0);
                nextPivotCommand = new Pivot(0);
                if (this.inTolerance(this.getElevatorInches(), -0.5, 0.5) && this.inTolerance(this.getPivotDegrees(), -2, 2)) {
                    state_ = ClimberStates.STABILIZE;
                }
                break;
            case STABILIZE:
                DrivetrainSubsystem d = DrivetrainSubsystem.getInstance();
                if (this.inTolerance(d.getGyroPitch(), -3, 3)) {
                    state_ = ClimberStates.IDLE;
                }
                break;
            default:
                nextPivotCommand = new Pivot(0);
                nextElevatorCommand = new Elevator(0);
                break;
        }
        nextPivotCommand.schedule();
        nextElevatorCommand.schedule();
        //#endregion

    }

    private void stopRoutine() {
        shouldRun_ = () -> false;
    }
    //#endregion

    //#region Commands
    public CommandGroupBase RaiseClimber() {
        return new RunCommand(() -> climberDeploy(true), this).withInterrupt(isClimberOut_);
    }

    public CommandGroupBase LowerClimber() {
        return new RunCommand(() -> climberDeploy(false), this).withInterrupt(isClimberOut_);
    }

    public CommandBase ExtendClimber() {
        return new InstantCommand(() -> setElevatorGoal(Constants.MAX_ELEVATOR_HEIGHT));
     }

     public CommandBase RetractClimber() {
        return new InstantCommand(() -> setElevatorGoal(-Constants.MAX_ELEVATOR_HEIGHT));
     }

    public CommandGroupBase StartRoutine() {
        return new RunCommand(this::runRoutine, this).withInterrupt(shouldRun_);
    }

    public CommandBase StopRoutine() {
        return new InstantCommand(this::stopRoutine, this);
    }
    //#endregion

    public static Climber getInstance() {
        if (instance_ == null) {
            instance_ = new Climber();
        }

        return instance_;
    }

    //Wrapper class for a WPILib Median Filter
    class VoltageTracker {

        private MedianFilter medianFilter_;
    
        public VoltageTracker(int sampleSize) {
            medianFilter_ = new MedianFilter(sampleSize);
        }
        
        public double calculate(double next) {
            return this.medianFilter_.calculate(next);
        }

        public void reset() {
            this.medianFilter_.reset();
        }
    }
}