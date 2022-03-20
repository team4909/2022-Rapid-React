package frc.lib.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import frc.lib.swervedrivespecialties.swervelib.*;
import frc.robot.utils.PIDGains;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import static frc.lib.swervedrivespecialties.swervelib.ctre.CtreUtils.checkCtreError;

public final class Falcon500SteerController {
    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private static final double TICKS_PER_ROTATION = 2048.0;

    // PID configuration
    private double proportionalConstant = Double.NaN;
    private double integralConstant = Double.NaN;
    private double derivativeConstant = Double.NaN;

    // Motion magic configuration
    private double velocityConstant = Double.NaN;
    private double accelerationConstant = Double.NaN;
    private double staticConstant = Double.NaN;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public Falcon500SteerController createFalconSteer(PIDGains gains, double currentLimit, double nominalVoltage) {
        this.proportionalConstant = gains.kP;
        this.integralConstant = gains.kI;
        this.derivativeConstant = gains.kD;
        this.nominalVoltage = nominalVoltage;
        this.currentLimit = currentLimit;

        return this;
    }
    

    // public void addDashboardEntries(ShuffleboardContainer container, SteerControllerImplementation controller) {
    //     SteerControllerFactory.super.addDashboardEntries(container, controller);
    //     container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(controller.absoluteEncoder.getAbsoluteAngle()));
    // }

    public SteerControllerImplementation create(Falcon500SteerConfiguration steerConfiguration, ModuleConfiguration moduleConfiguration) {
        CanCoderAbsolute absoluteEncoder = new CanCoderAbsolute(100, steerConfiguration.getEncoderConfiguration());

        final double sensorPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * moduleConfiguration.getSteerReduction();
        final double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.slot0.kP = proportionalConstant;
        motorConfiguration.slot0.kI = integralConstant;
        motorConfiguration.slot0.kD = derivativeConstant;
        
        motorConfiguration.voltageCompSaturation = nominalVoltage;
        
        motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;
        

        TalonFX motor = new TalonFX(steerConfiguration.getMotorPort());
        checkCtreError(motor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS), "Failed to configure Falcon 500 settings");

        motor.enableVoltageCompensation(true);
        
        checkCtreError(motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS), "Failed to set Falcon 500 feedback sensor");
        motor.setSensorPhase(true);
        motor.setInverted(moduleConfiguration.isSteerInverted() ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
        motor.setNeutralMode(NeutralMode.Brake);

        checkCtreError(motor.setSelectedSensorPosition(absoluteEncoder.getAbsoluteAngle() / sensorPositionCoefficient, 0, CAN_TIMEOUT_MS), "Failed to set Falcon 500 encoder position");

        // Reduce CAN status frame rates
        CtreUtils.checkCtreError(
                motor.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_1_General,
                        STATUS_FRAME_GENERAL_PERIOD_MS,
                        CAN_TIMEOUT_MS
                ),
                "Failed to configure Falcon status frame period"
        );

        return new SteerControllerImplementation(motor,
                sensorPositionCoefficient,
                sensorVelocityCoefficient,
                TalonFXControlMode.Position,
                absoluteEncoder);
    
    }

    public static class SteerControllerImplementation { // implements SteerController {
        private static final int ENCODER_RESET_ITERATIONS = 500;
        private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

        private final TalonFX motor;
        private final double motorEncoderPositionCoefficient;
        private final double motorEncoderVelocityCoefficient;
        private final TalonFXControlMode motorControlMode;
        private final CanCoderAbsolute absoluteEncoder;

        private double referenceAngleRadians = 0.0;

        private double resetIteration = 0;

        private SteerControllerImplementation(TalonFX motor,
                                         double motorEncoderPositionCoefficient,
                                         double motorEncoderVelocityCoefficient,
                                         TalonFXControlMode motorControlMode,
                                         CanCoderAbsolute absoluteEncoder) {
            this.motor = motor;
            this.motorEncoderPositionCoefficient = motorEncoderPositionCoefficient;
            this.motorEncoderVelocityCoefficient = motorEncoderVelocityCoefficient;
            this.motorControlMode = motorControlMode;
            this.absoluteEncoder = absoluteEncoder;
        }

        // @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        // @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            double currentAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;

            // Reset the NEO's encoder periodically when the module is not rotating.
            // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
            // end up getting a good reading. If we reset periodically this won't matter anymore.
            if (motor.getSelectedSensorVelocity() * motorEncoderVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
                if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                    resetIteration = 0;
                    double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
                    motor.setSelectedSensorPosition(absoluteAngle / motorEncoderPositionCoefficient);
                    currentAngleRadians = absoluteAngle;
                }
            } else {
                resetIteration = 0;
            }

            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
            double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
            if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
            } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
                adjustedReferenceAngleRadians += 2.0 * Math.PI;
            }

            motor.set(motorControlMode, adjustedReferenceAngleRadians / motorEncoderPositionCoefficient);


            this.referenceAngleRadians = referenceAngleRadians;
        }

        // @Override
        public double getStateAngle() {
            double motorAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }

            return motorAngleRadians;
        }
    }
}
