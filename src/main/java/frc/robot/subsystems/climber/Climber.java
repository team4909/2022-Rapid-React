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
import com.kauailabs.navx.AHRSProtocol.IntegrationControl;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    public boolean holdingPivot_;

    private ClimberStates state_;

    private VoltageTracker voltageTracker_;

    public BooleanSupplier isClimberOut_;    
    private BooleanSupplier shouldRun_;

    private ShuffleboardLayout climberLayout;
    private NetworkTableEntry stateEntry;
    private NetworkTableEntry pivotPos;
    private NetworkTableEntry elevatorPos;
    private NetworkTableEntry pivotVoltage;

    DrivetrainSubsystem d = DrivetrainSubsystem.getInstance();


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
        //TODO CHANGE
        elevatorLeft_ = new CANSparkMax(Constants.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorRight_ = new CANSparkMax(Constants.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorRight_.setInverted(true);
        elevatorLeft_.follow(elevatorRight_, true);
        elevatorLeft_.clearFaults();
        elevatorRight_.clearFaults();
        elevatorRight_.getEncoder().setPosition(0);
        elevatorLeft_.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);
        elevatorRight_.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);

        elevatorController_ = elevatorRight_.getPIDController();
        elevatorController_.setP(Constants.ELEVATOR_KP, 0);
        elevatorController_.setD(Constants.ELEVATOR_KD, 0);
        elevatorController_.setI(Constants.ELEVATOR_KI, 0);
        elevatorController_.setFF(Constants.ELEVATOR_KF, 0);
    
        elevatorController_.setP(Constants.DOWN_ELEVATOR_KP, 1);
        elevatorController_.setD(Constants.DOWN_ELEVATOR_KD, 1);
        elevatorController_.setI(Constants.DOWN_ELEVATOR_KI, 1);
        elevatorController_.setFF(Constants.DOWN_ELEVATOR_KF, 1);


        pivotRight_ = new TalonFX(Constants.RIGHT_PIVOT_MOTOR);
        pivotLeft_ = new TalonFX(Constants.LEFT_PIVOT_MOTOR);
        pivotRight_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 200);
        pivotLeft_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 200);

        pivotRight_.clearStickyFaults(kTimeoutMs);
        // pivotLeft_.follow(pivotRight_);
        pivotLeft_.setInverted(InvertType.InvertMotorOutput);
        
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

        isClimberOut_ = () -> false;
        shouldRun_ = () -> true;

        voltageTracker_ = new VoltageTracker(10);
        state_ = ClimberStates.IDLE;


        //0.76

    }

    public void periodic() {

        // elevatorController_.setReference(10, ControlType.kPosition);
        // elevatorRight_.set(0.3);
        SmartDashboard.putNumber("posel", elevatorRight_.getEncoder().getPosition());
        SmartDashboard.putNumber("piv vol", pivotRight_.getMotorOutputVoltage());
        SmartDashboard.putNumber("piv pos", pivotRight_.getSelectedSensorPosition());
        SmartDashboard.putBoolean("is climber out", isClimberOut_.getAsBoolean());
        
        // System.out.println("R"+pivotRight_.getSelectedSensorPosition());
        // System.out.println("L"+pivotLeft_.getSelectedSensorPosition());
        stateEntry.setString(state_.toString());
        pivotPos.setDouble(pivotRight_.getSelectedSensorPosition());
        elevatorPos.setDouble(elevatorRight_.getEncoder().getPosition());
        pivotVoltage.setDouble(pivotRight_.getMotorOutputVoltage());
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

    public void setElevatorGoal(double goal, int slot) {
        System.out.println("Runnign");
        elevatorController_.setReference(goal, ControlType.kPosition, slot);
        // elevatorController_.setReference(goal * Constants.TICKS_PER_ELEVATOR_INCH, ControlType.kPosition);
    }

    //temp method for manual climb
    //used for getting off the bar so we can back pivot
    public void detach() {
        setElevatorGoal(10, 0);
    }

    public void stopEl() {
        elevatorLeft_.set(0);
        elevatorRight_.set(0);
    }

    public void setState(ClimberStates state) {
        state_ = state;
    }

    /**
     * @param out true will extend outwards (up), false will extend inwards (down)
     */
    private void climberDeploy(boolean out) {
        pivotRight_.set(ControlMode.PercentOutput, out ? -0.15 : 0.15); //Deploy at 30%
        pivotLeft_.set(ControlMode.PercentOutput, out ? -0.15 : 0.15); //Deploy at 30%
        double avgV = voltageTracker_.calculate(pivotRight_.getMotorOutputVoltage());
        isClimberOut_ = () -> (Math.abs(pivotRight_.getMotorOutputVoltage()) > 1); // (Math.abs(avgV) * 10);
        if (isClimberOut_.getAsBoolean()) {
            System.out.println(pivotRight_.getSelectedSensorPosition());
            voltageTracker_.reset();
            pivotRight_.setSelectedSensorPosition(0); // Zero the pivot motors
            pivotLeft_.setSelectedSensorPosition(0); // Zero the pivot motors

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
                nextElevatorCommand = new Elevator(26.5);
                if (this.inTolerance(this.getElevatorInches(), 26, 27)) { // extension tolerance of 1/2 in
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

    // public void holdPivot() {
    //     pivotRight_.set(TalonFXControlMode.Position, 0);
    // }

    private void stopRoutine() {
        shouldRun_ = () -> false;
    }
    //#endregion

    //#region Commands
    public CommandGroupBase RaiseClimber() {
        holdingPivot_ = true;
        return new RunCommand(() -> climberDeploy(true), this).withTimeout(1).andThen(this.HoldClimber()); // .withInterrupt(this.isClimberOut_);
    }

    public CommandGroupBase LowerClimber() {
        isClimberOut_ = () -> false;
        holdingPivot_ = false;
        return new RunCommand(() -> climberDeploy(false), this).withTimeout(1).andThen(this.HoldClimber());
    }

    public RunCommand HoldClimber() {
    //    return  new SequentialCommandGroup(
            // new PrintCommand("setting to 0"),
        pivotRight_.setSelectedSensorPosition(0);
        pivotLeft_.setSelectedSensorPosition(0);
        return new RunCommand(() -> {pivotRight_.set(TalonFXControlMode.Position, 0); pivotLeft_.set(TalonFXControlMode.Position, 0);}); //.withInterrupt(() -> this.holdingPivot_)

    }

    public CommandBase ExtendClimber() {
        // elevatorController_.setP(Constants.ELEVATOR_KP);
        // elevatorController_.setD(Constants.ELEVATOR_KD);
        // elevatorController_.setI(Constants.ELEVATOR_KI);
        // elevatorController_.setFF(Constants.ELEVATOR_KF);
        return new RunCommand(() -> setElevatorGoal(36.5, 0)).withInterrupt(() -> inTolerance(elevatorRight_.getEncoder().getPosition(), 35, 36)).andThen(new WaitCommand(0.5)).andThen(new InstantCommand(this::stopEl));
 
     }

     public CommandBase RetractClimber() {
        stopTalons();
        // elevatorController_.setP(Constants.DOWN_ELEVATOR_KP);
        // elevatorController_.setD(Constants.DOWN_ELEVATOR_KD);
        // elevatorController_.setI(Constants.DOWN_ELEVATOR_KI);
        // elevatorController_.setFF(Constants.DOWN_ELEVATOR_KF);
        return new RunCommand(() -> setElevatorGoal(0.025, 1)).withInterrupt(() -> inTolerance(elevatorRight_.getEncoder().getPosition(), 0, 1)).andThen(new WaitCommand(0.5)).andThen(new InstantCommand(this::stopEl));
    
    }

    public CommandGroupBase StartRoutine() {
        holdingPivot_ = false;
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

    //TODO KILL THIS LATER
    public void stopTalons() {
        pivotLeft_.set(ControlMode.PercentOutput, 0);
        pivotRight_.set(ControlMode.PercentOutput, 0);
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