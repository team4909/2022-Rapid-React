package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.Rumble;

public class IntakeFeeder extends SubsystemBase {

    private enum IntakeState {
        kIdle("Idle"),
        kAdjust("Adjust Balls"),
        kIdleFeeder("Idle Feeder"),
        kIntakeFirst("Intake First"),
        kIntakeSecond("Intake Second"),
        kShootBalls("Shooting Both Balls"),
        kReverseWrongBall("Reverse Wrong Ball");

        public String name;

        private IntakeState(String n) {
            this.name = n;
        }

        public String toString() {
            return this.name;
        }
    };

    private enum BallCount{
        
        kZero("ZERO"), kOne("ONE"), kTwo("TWO");
        String m_count;

        private BallCount(String c) {
            this.m_count = c;
        }

        public String toString() {
            return this.m_count;
        }
    }

    // Object instance
    private static IntakeFeeder instance_ = null;

    private CANSparkMax intakeWheels_;
    private CANSparkMax centeringWheel_;
    private CANSparkMax feederWheel_;

    // private final DoubleSolenoid intakeExtension_;

    private final DigitalInput lowSensor_; // One closest to intake
    private final DigitalInput highSensor_; // One closes to shooter
    private final DigitalInput incomingSensor_; // To know when to run feader

    // private static boolean intakeSolenoidState_ = false; // Assuming false is the default state (intact retracted)

    private static IntakeState currentState_;
    private static IntakeState lastState_;

    private BallCount ballsHeld_;

    private static int fallingEdges = 0;
    private static boolean lastEdgeHigh = false;
    private static boolean rumble_ = false;
    private static boolean adjusted_ = false;

    private static Timer shot_timer_;



    private IntakeFeeder() {
        intakeWheels_ = new CANSparkMax(18, MotorType.kBrushless);
        centeringWheel_ = new CANSparkMax(16, MotorType.kBrushless);
        feederWheel_ = new CANSparkMax(17, MotorType.kBrushless);

        intakeWheels_.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);
        centeringWheel_.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);
        feederWheel_.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);

        feederWheel_.setIdleMode(IdleMode.kBrake);

        // intakeExtension_ = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    
        incomingSensor_ = new DigitalInput(2);
        lowSensor_ = new DigitalInput(1);
        highSensor_ = new DigitalInput(0);

        currentState_ = IntakeState.kIdle;
        lastState_ = IntakeState.kIdle;

        ballsHeld_ = BallCount.kZero;

        shot_timer_ = new Timer();
        shot_timer_.reset();
    }

    public static IntakeFeeder getInstance() {
        if (instance_ == null) {
            instance_ = new IntakeFeeder();
        } 
        return instance_;
    }

    public void intake() {
        // intakeSolenoidState_ = true;

        if (currentState_.equals(IntakeState.kShootBalls)) {
            return;
        }

        Intake.getInstance().intakeOut();

        switch (ballsHeld_) {
            case kZero:
                currentState_ = IntakeState.kIntakeFirst; // Change back to intake first
                break;
            case kOne:
                currentState_ = IntakeState.kIdleFeeder;
                break;
            case kTwo:
            default:
                currentState_ = IntakeState.kIdleFeeder;
                break;
        }
    }

    // Stop all rollers in the intake
    public void stopIntake() {
        currentState_ = IntakeState.kIdle;
        Intake.getInstance().intakeIn();
        // intakeSolenoidState_ = false; // intake in

    }

    // Tell the feeder system to run all the motors to shoot
    public void shoot() {
        currentState_ = IntakeState.kShootBalls;
        shot_timer_.reset();
        shot_timer_.start();
    }

    // Manually toggle the intake moving in and out (might not be needed, but nice to have)
    public void toggleIntakeExtension() {
        Intake.getInstance().intakeOut();
        // intakeSolenoidState_ = !intakeSolenoidState_;
    }

    // Right now map this to a command/button. can make automatic with the color sensor later
    // Just want to test that reversing works
    public void reverseIntake() {
        currentState_ = IntakeState.kReverseWrongBall;
        Intake.getInstance().intakeIn();
        // intakeSolenoidState_ = !intakeSolenoidState_; // intake in

    }

    public boolean getRumble() {
        return rumble_;
    }

    public void compressBalls() {
        feederWheel_.set(-0.2);
        
    }

    @Override
    public void periodic() {

        BallCount lastBallsHeld = ballsHeld_;

        // Get all the readings from the sensors each tick so we know what's changed
        boolean firstBallSeen = !lowSensor_.get();
        boolean feederBallSeen = !highSensor_.get();
        boolean incomingSeen = !incomingSensor_.get();
        //Smart Dashboard outputs
        SmartDashboard.putBoolean("First Ball Seen", firstBallSeen);
        SmartDashboard.putBoolean("Feeder Ball Seen", feederBallSeen);
        SmartDashboard.putBoolean("Incoming Seen", incomingSeen);

        SmartDashboard.putString("State", currentState_.toString());
        // Set the solenoid to be in it's state
        // Because it's a single acting solenoid it needs to be continously set
        // otherwise it reverts to the default state
        // intakeExtension_.set(intakeSolenoidState_ ? Value.kReverse : Value.kForward);
        
        if (ballShot(feederBallSeen)) {
            ballsHeld_ = (ballsHeld_ == BallCount.kTwo) ? BallCount.kOne : BallCount.kZero;
        }
        
        switch (currentState_) {
            case kIdle:
                // Everything is idle, nothing runs
                intakeWheels_.set(0.0);
                centeringWheel_.set(0.0);
                feederWheel_.set(0.0);

                shot_timer_.stop();
                rumble_ = false;

                if (!adjusted_ && feederBallSeen) {
                    currentState_ = IntakeState.kAdjust;
                    shot_timer_.reset();
                    shot_timer_.start();
                }      
                break;
            case kAdjust:
                intakeWheels_.set(0.0);
                centeringWheel_.set(0.0);
                 if (shot_timer_.get() < 1.0 ) {
                    feederWheel_.set(0.0);  
                } else if (shot_timer_.get() < 1.5) {
                    feederWheel_.set(Constants.kFeederAdjustVoltage);
                } else {
                    feederWheel_.set(0.0);  
                    shot_timer_.stop();
                    currentState_ = IntakeState.kIdle;
                    adjusted_ = true;
                }
                rumble_ = false;
                break;
            case kIdleFeeder:
                // Only go to this state between intaking balls one and two. Could be better named
                // to "IntermediateIntake" or something if that's better
                feederWheel_.set(0.0);
                // Continue running the intaking and centering wheels until there's another ball seen
                intakeWheels_.setVoltage(Constants.kIntakeForwardVoltage);
                centeringWheel_.setVoltage(Constants.kCenteringWheelForwardVoltage);
                if (incomingSeen && !feederBallSeen) {
                    // Set state to get the next ball
                    currentState_ = IntakeState.kIntakeSecond;
                }
                rumble_ = false;
                adjusted_ = false;
                break;
            case kIntakeFirst:
                // Run everything to get first ball into position
                intakeWheels_.setVoltage(Constants.kIntakeForwardVoltage);
                centeringWheel_.setVoltage(Constants.kCenteringWheelForwardVoltage);
                feederWheel_.setVoltage(Constants.kFeederFeedingVoltage);
                if (firstBallSeen) {
                    // Want to keep running the intake, but not the feeder
                    currentState_ = IntakeState.kIdleFeeder;
                    ballsHeld_ = BallCount.kOne;
                    // Not doing this yet, but for the future driver feedback
                    rumble_ = true;
                } 
                if (feederBallSeen) {
                    currentState_ = IntakeState.kIdleFeeder;
                    ballsHeld_ = BallCount.kTwo;
                    // Not doing this yet, but for the future driver feedback
                    rumble_ = false;
                }  
                adjusted_ = false;
                break;
            case kIntakeSecond:
                // Keep running intake, but now know the second ball is at least in the robot
                intakeWheels_.setVoltage(Constants.kIntakeForwardVoltage);
                centeringWheel_.setVoltage(Constants.kCenteringWheelForwardVoltage);
                // Have already checked to make sure that the fender wheel should be running                
                feederWheel_.setVoltage(Constants.kFeederFeedingVoltage);
                // Stop feeder if either two balls reach the position
                // Might need more complex logic, depends on the geometry but should be fine for testing
                if (feederBallSeen ) {
                    currentState_ = IntakeState.kIdle;
                    ballsHeld_ = BallCount.kTwo;
                    // Not doing this yet, but for the future driver feedback
                    rumble_ = false;
                }
                adjusted_ = false;
                break;
            case kShootBalls:
                intakeWheels_.set(0.0);
                centeringWheel_.set(0.0);
                // if (shot_timer_.get() < 0.1 || shot_timer_.get() > 0.5) {
                    feederWheel_.setVoltage(Constants.kFeederShootingVoltage);
                // } else {
                //     feederWheel_.setVoltage(0.0);
                // }

                ballsHeld_ = BallCount.kZero;
                adjusted_ = false;
                break;
            case kReverseWrongBall:
                // Reverse the motors
                ballsHeld_ = BallCount.kZero; // TODO fix with actually counting balls with sensors
                intakeWheels_.setVoltage(Constants.kIntakeReverseVoltage);
                centeringWheel_.setVoltage(Constants.kCenteringWheelReverseVoltage);
                feederWheel_.setVoltage(Constants.kFeederReverseVoltage);
                rumble_ = false;
                adjusted_ = false;
                break;
            default:
                break;
        }
        SmartDashboard.putString("Intake State", currentState_.name);
        SmartDashboard.putString("Balls Held", ballsHeld_.toString());

        // not using this now but probably will want to down the line
        // for knowing what state the robot previously was in
        lastState_ = currentState_;
        lastEdgeHigh = feederBallSeen;


        if (!lastBallsHeld.toString().equals(ballsHeld_.toString())){
            Rumble.getInstance().runRumble(3, 1, 1, 0, 1).schedule();
        }


    }

    public void resetBallCount() {
        this.ballsHeld_ = BallCount.kZero;
    }

    private boolean ballShot(boolean seen) {
        return lastEdgeHigh && !seen;
    }

}