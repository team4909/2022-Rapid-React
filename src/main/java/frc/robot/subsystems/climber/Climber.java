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
    
    private ClimberStates state_;

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
                
                if (pivotRight_.getMotorOutputVoltage() > getLastVoltages()) { //THIS PROBABLY ISNT THE RIGHT MAKE IT BETTER PLEASEEE
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
        //#endregion

        SmartDashboard.putString("Current State", state_.toString());
        nextPivotCommand.schedule();
        nextElevatorCommand.schedule();
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

    public double getLastVoltages() {
        ArrayList<Double> lastVoltages = new ArrayList<Double>(10);
        int vI = 0;
        if (vI >= 10) vI = 0;
        lastVoltages.set(vI, pivotRight_.getMotorOutputVoltage());
        double avgVoltage = 0;
        for(int i = 0; i < lastVoltages.size(); i++)
            avgVoltage += lastVoltages.get(i);
        return avgVoltage / lastVoltages.size();
    }

    public static Climber getInstance() {
        if (instance_ == null) {
            instance_ = new Climber();
        }

        return instance_;
    }
    //#endregion
}