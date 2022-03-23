package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;

public class InterpolationTable {
    private final ArrayList<Pair<Double, Double>> m_table = new ArrayList<>();

    public InterpolationTable add(double x, double y) {
        m_table.add(Pair.of(x, y));
        return this;
    }

    private double interpolate(Pair<Double, Double> l, Pair<Double, Double> h, double value) {
        double ratio = (value - l.getFirst()) / (h.getFirst() - l.getFirst());
        return l.getSecond() + ratio * (h.getSecond() - l.getSecond());
    }

    public double get(double value) {

        // return lowest possible value
        if (value < m_table.get(0).getFirst()) {
            return m_table.get(0).getSecond();
        } 

        for (int ii = 0; ii < m_table.size() - 1; ii++) {
            Pair<Double, Double> high = m_table.get(ii + 1);
            Pair<Double, Double> low = m_table.get(ii);

            if (value < high.getFirst() && value > low.getFirst()) {
                return interpolate(low, high, value);
            }
        }

        // return highest possible value
        return m_table.get(m_table.size() - 1).getSecond();
        
    }
 }
