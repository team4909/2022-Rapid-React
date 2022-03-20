package frc.robot.subsystems.intake;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Optional;

public class BallColorSensor implements Sendable {

	// Constants
	private static final double kThresholdMillimeters = 40.0; // millimeters
	private static boolean isVisable_;
	private final ColorSensorV3 colorSensor_;

	public BallColorSensor() {
		colorSensor_ = new ColorSensorV3(Port.kMXP);
		colorSensor_.configureColorSensor(
				ColorSensorResolution.kColorSensorRes16bit,
				ColorSensorMeasurementRate.kColorRate25ms,
				GainFactor.kGain6x);
		colorSensor_.configureProximitySensor(
				ProximitySensorResolution.kProxRes10bit, ProximitySensorMeasurementRate.kProxRate12ms);

		isVisable_ = false;
	}

	public Optional<Color> getColor() {
		if (getDistanceMillimeters().isEmpty()) {
			return Optional.empty();
		}

		var color = colorSensor_.getRawColor();

		// Add extra gain to 'equal out' the spectral response. See Figure 1. of the APDS-9151 datasheet
		double blue = (double) color.blue * 1.666666;
		double red = (double) color.red * 1.11111111;

		if (red > blue) {
			return Optional.of(Color.kRed);
		} else {
			return Optional.of(Color.kBlue);
		}

	}

	private Optional<Double> getDistanceMillimeters() {
		double distance = colorSensor_.getProximity();

		if (distance < kThresholdMillimeters) {
			isVisable_ = true;
		} else if (distance > kThresholdMillimeters + 5.0) {
			isVisable_ = false;
		}

		if (isVisable_) {
			return Optional.of(distance);
		}

		return Optional.empty();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		// TODO Auto-generated method stub
	}
}
