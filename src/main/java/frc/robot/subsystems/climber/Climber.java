package frc.robot.subsystems.climber;

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

    enum ClimberStates {
        IDLE, 
        PIVOT_FORWARD, 
        EXTEND_HOOK, 
        BACK_PIVOT, 
        RETRACT, 
        STABILIZE;
    }

    private Climber() {
        //Elevator: NEOs (CANSparkMaxs)
        //Pivot: Falcons (TalonFXs)
        //TODO Configure these
        //#region Motor Config
        elevatorRight_ = new CANSparkMax(Constants.RIGHT_ELEVATOR_MOTOR, null);
        elevatorLeft_ = new CANSparkMax(Constants.LEFT_ELEVATOR_MOTOR, null);
        elevatorLeft_.follow(elevatorRight_, true);

        elevatorController_ = elevatorLeft_.getPIDController();

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

        switch (state_) {
            case IDLE:
                nextPivotCommand = new Pivot(0);
                nextElevatorCommand = new Elevator(0);
                break;
            case PIVOT_FORWARD:
                nextPivotCommand = new Pivot(Constants.BAR_THETA); //FIXME assign value, overshoot a little
                nextElevatorCommand = new Elevator(0);
                if (elevatorLeft_.getEncoder().getPosition() < 00 || elevatorLeft_.getEncoder().getPosition() < 00) { //FIXME Add values, degrees of tolerence
                    state_ = ClimberStates.EXTEND_HOOK;
                }
                break;
            case EXTEND_HOOK:
                nextPivotCommand = new Pivot(0);
                nextElevatorCommand = new Elevator(00); //FIXME Find elevator setpoint

                //elevate
                if (m_isAtElevatorGoal) {
                    state_ = ClimberStates.BACK_PIVOT;
                }
                break;
            case BACK_PIVOT:
                nextElevatorCommand = new Elevator(0);
                nextPivotCommand = new Pivot(00); //FIXME backPivotGreaterThanHorizontal slam against bar to prepare for retraction
                if (elevatorLeft_.getVoltageCompensationNominalVoltage() == 0) { //FIXME check for voltage spike ALSO THIS PROBABLY ISNT THE RIGHT METHOD!
                    state_ = ClimberStates.RETRACT;
                }
                break;                
            case RETRACT:
                nextElevatorCommand = new Elevator(00); //FIXME value to retract all the way
                nextPivotCommand = new Pivot(0);
                if (m_isAtElevatorGoal && m_isAtPivotGoal) {
                    state_ = ClimberStates.STABILIZE;
                }
                break;
            case STABILIZE:
                DrivetrainSubsystem d = DrivetrainSubsystem.getInstance();
                //TODO test these values.
                if (d.getGyroPitch() < 5 && d.getGyroPitch() > -5) {
                    state_ = ClimberStates.IDLE;
                }
                break;
            default:
                nextPivotCommand = new Pivot(0);
                nextElevatorCommand = new Elevator(0);
                break;
        }

        SmartDashboard.putString("Current State", state_.toString());
        nextPivotCommand.schedule();
        nextElevatorCommand.schedule();
    }

    //TODO setSpeed, stop, etc...

    
    public void setPivotGoal(double goal) {
        elevatorController_.setReference(goal, ControlType.kPosition);
        //FIXME implement
        //including PID logic, isAtGoal logic
    } 
    public void setElevatorGoal(double goal) {
        
        //TODO implement
    }

    public static Climber getInstance() {
        if (instance_ == null) {
            instance_ = new Climber();
        }

        return instance_;
    }
}