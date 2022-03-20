package frc.robot.utils;

public class PIDGains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;

	public PIDGains(double p, double i, double d) {
		this(p, i, d, 0);
	}

	public PIDGains(double p, double i, double d, double f) {
		kP = p;
		kI = i;
		kD = d;
		kF = f;
	}
}
