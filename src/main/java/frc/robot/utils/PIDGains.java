package frc.robot.utils;

public class PIDGains {
    public PIDGains(double p, double i, double d) {
        this(p, i, d, 0);
    }

    public PIDGains(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }

    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kF = 0.0;
}
