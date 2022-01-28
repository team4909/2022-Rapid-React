package frc.robot.subsystems.intake;

import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private enum IntakeState {
        kIdle("Idle"),
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
        kZero, kOne, kTwo
    }

    // Object instance
    private static Intake instance_;

    private final Neo550Roller intakeWheels_;
    private final Neo550Roller centeringWheel_;
    private final Neo550Roller feederWheel_;

    private final DoubleSolenoid intakeExtension_;

    private final DigitalInput grabSensor_; // One closest to intake
    private final DigitalInput stopSensor_; // One closes to shooter


    private final BallColorSensor ballColorSensor_;

    private static IntakeState currentState_;
    private static IntakeState lastState_;

    private static BallCount ballsHeld_;

    private static int fallingEdges = 0;
    private static boolean lastEdgeHigh = false;


    // Constants



    private Intake() {
        intakeWheels_ = new Neo550Roller(20, false);
        centeringWheel_ = new Neo550Roller(21, false);
        feederWheel_ = new Neo550Roller(22, false);

        intakeExtension_ = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    
        grabSensor_ = new DigitalInput(0);
        stopSensor_ = new DigitalInput(1);

        ballColorSensor_ = new BallColorSensor();

        currentState_ = IntakeState.kIdle;
        lastState_ = IntakeState.kIdle;

        ballsHeld_ = BallCount.kZero;
    }

    public static Intake getInstance() {
        if (instance_ == null) {
            instance_ = new Intake();
        } 
        return instance_;
    }

    public void intake() {
        switch (ballsHeld_) {
            case kZero:
                currentState_ = IntakeState.kIntakeFirst;
                break;
            case kOne:
                currentState_ = IntakeState.kIntakeSecond;
                break;
            case kTwo:
            default:
                currentState_ = IntakeState.kIdle;
                break;
        }
    }

    public void shoot() {
        currentState_ = IntakeState.kShootBalls;
    }

    @Override
    public void periodic() {

        boolean grabBallSeen = grabSensor_.get();
        boolean feederBallSeen = stopSensor_.get();
        Optional<Color> seenColor = ballColorSensor_.getColor();
        
        switch (currentState_) {
            case kIdle:
                intakeWheels_.stop();
                centeringWheel_.stop();
                feederWheel_.stop();
                break;
            case kIntakeFirst:
                intakeWheels_.run(Constants.kIntakeForwardVoltage);
                centeringWheel_.run(Constants.kCenteringWheelForwardVoltage);
                feederWheel_.stop();
                if (grabBallSeen) {
                    currentState_ = IntakeState.kIdle;
                    ballsHeld_ = BallCount.kOne;
                }
                break;
            case kIntakeSecond:
                intakeWheels_.run(Constants.kIntakeForwardVoltage);
                centeringWheel_.run(Constants.kCenteringWheelForwardVoltage);
                feederWheel_.run(Constants.kFeederFeedingVoltage);
                if (grabBallSeen || feederBallSeen) {
                    currentState_ = IntakeState.kIdle;
                    ballsHeld_ = BallCount.kTwo;
                    lastEdgeHigh = feederBallSeen;
                }
                break;
            case kShootBalls:
                intakeWheels_.stop();
                centeringWheel_.run(Constants.kCenteringWheelForwardVoltage);
                feederWheel_.run(Constants.kFeederShootingVoltage);
                if(countFalling(grabBallSeen) >= 2) {
                    fallingEdges = 0;
                    ballsHeld_ = BallCount.kZero;
                    currentState_ = IntakeState.kIdle;
                }
                
                break;
            case kReverseWrongBall:
                intakeWheels_.run(Constants.kIntakeReverseVotlage);
                centeringWheel_.run(Constants.kCenteringWheelReverseVotlage));
                break;
            default:
                break;
        }

        lastState_ = currentState_;
    }

    private int countFalling(boolean seen) {
        if (lastEdgeHigh && lastEdgeHigh != seen) {
            fallingEdges++;
            lastEdgeHigh = false;
        }

        return fallingEdges;
    }

}