package frc.robot.subsystems.climber;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.commands.Pivot;

public class Climber extends SubsystemBase {
    

    private static Climber m_instance = null;
    public String state;
    
    private CANSparkMax pivot_left;
    private CANSparkMax pivot_right;

    private CANSparkMax climber_left;
    private CANSparkMax climber_right;

    private SparkMaxPIDController pivotController;
    private SparkMaxPIDController elevatorController;


    //isDone flags
    public boolean isAtPivotGoal;
    public boolean isAtElevatorGoal;
    

    private Climber() {
        //TODO Configure these
        pivot_left = new CANSparkMax(0, null);
        pivot_right = new CANSparkMax(0, null);
        pivot_right.follow(pivot_left, true);

        climber_left = new CANSparkMax(0, null);
        climber_right = new CANSparkMax(0, null);

        pivotController = pivot_left.getPIDController();
    }

    
    private enum STATES {
        
    
    }

    @Override
    public void periodic() {
        CommandBase nextPivotCommand;
        CommandBase nextElevatorCommand;
        switch (state) {
            case "IDLE":
                nextPivotCommand = new Pivot(0);
                nextElevatorCommand = new Elevator(0);
                break;
            case "PIVOT_FORWARD":
                nextPivotCommand = new Pivot(overshoot a little);
                nextElevatorCommand = new Elevator(0);
                if (pivot_left.getEncoder().getPosition() < degTol || pivot_left.getEncoder().getPosition() < degTol) {
                    state = "PIVOT_REACHED";
                }
                break;
            case "EXTEND_HOOK":
                nextCommand = new Pivot(0);
                nextElevatorCommand = new Elevator(setpoint);

                //elevate
                if (atElevatorSetpoiont) {
                    state = "BACK_PIVOT";
                }
                break;
            case "BACK_PIVOT":
                nextElevatorCommand = new Elevator(0);
                nextPivotCommand = new Pivot(-backPivotGreaterThanHorizontal); //slam against bar to prepare for retraction
                if (currentTrip) {
                    state = "RETRACT";
                }
                break;                
            case "RETRACT":
                nextElevatorCommand = new Elevator(up);
                nextPivotCommand = new Pivot(-backPivot);
                if (atElevatorSetpoiont && pivotHorizontalGoal) {
                    state = "STABILIZE";
                }
            case "STABILIZE":
                if (pitch == 0) {
                    state = "IDLE";
                }
            default:
                break;
        }

        nextPivotCommand.schedule();
    }

    //TODO setSpeed, stop, etc...

    
    public void setPivotGoal(double goal) {
        pivotController.setReference(goal, ControlType.kPosition);
        //TODO implement
        //including PID logic, isAtGoal logic
    } 
    public void setElevatorGoal(double goal) {
        //TODO implement
    }

    public static Climber getInstance() {
        if (m_instance == null) {
            m_instance = new Climber();
        }

        return m_instance;
    }
}

