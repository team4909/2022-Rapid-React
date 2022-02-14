package frc.robot.subsystems.climber;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.commands.Elevator;
import frc.robot.subsystems.climber.commands.Pivot;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class Climber extends SubsystemBase {

    private static Climber instance_ = null;
    private static final int kTimeoutMs = 100;
    private static final double kVelocityConversion = 600 / 2048d;
    
    private CANSparkMax elevatorLeft_;
    private CANSparkMax elevatorRight_;

    private TalonFX pivotLeft_;
    private TalonFX pivotRight_;

    private SparkMaxPIDController elevatorController_;

    //isDone flags
    public boolean m_isAtPivotGoal;
    public boolean m_isAtElevatorGoal;
    
    private boolean start_ = false;

    private ClimberStates state_;

    private VoltageTracker vc;

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
        state_ = ClimberStates.IDLE;


    }

    @Override
    public void periodic() {
        CommandBase nextPivotCommand = null;
        CommandBase nextElevatorCommand = null;

        if (start_ = true) {
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
                if (vc == null) new VoltageTracker(10);
                vc.add(pivotRight_.getMotorOutputVoltage());
                if (vc.sampleFull == true) {
                    if (pivotRight_.getMotorOutputVoltage() > (vc.getAverage() * 2)) { //If the current voltage is equal to double the average before it
                        vc = null; //Get rid of current Voltage Tracker instance
                        state_ = ClimberStates.RETRACT;
                    }
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
    
        //#endregion

        SmartDashboard.putString("Current State", state_.toString());
        nextPivotCommand.schedule();
        nextElevatorCommand.schedule();
        }
        start_ = false;
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

    public void climberDeploy() {
        pivotRight_.set(ControlMode.PercentOutput, 0.3); //Deploy at 30%
        if (vc == null) new VoltageTracker(10);
        vc.add(pivotRight_.getMotorOutputVoltage());
        if (vc.sampleFull == true) {
            if (pivotRight_.getMotorOutputVoltage() > (vc.getAverage() * 2)) { //If the current voltage is equal to double the average before it
                vc = null; //Get rid of current Voltage Tracker instance
            }
        }

    }

    // public double getLastVoltages() {


    // }

    public void setState(ClimberStates state) {
        state_ = state;
    }
    public void start() {
        start_ = true;
    }

    public static Climber getInstance() {
        if (instance_ == null) {
            instance_ = new Climber();
        }

        return instance_;
    }
    //#endregion
}


class VoltageTracker {

    private int sampleSize;
    private double total = 0d;
    private int index = 0;
    private double samples[];

    public boolean sampleFull = false;

    public VoltageTracker(int sampleSize) {
        this.sampleSize = sampleSize;
        samples = new double[sampleSize];
        for (int i = 0; i < sampleSize; i++) samples[i] = 0d;
    }

    public void add(double x) {
        total -= samples[index];
        samples[index] = x;
        total += x;
        if (++index == sampleSize) {
            index = 0;
            sampleFull = true;
        }
    }

    public double getAverage() {
        return total / sampleSize;
    }   

    
}