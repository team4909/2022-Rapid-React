package frc.robot.utils;

/**
 * Conversion
 */
public class Conversion {

    /**
     * @param inches
     *  Value to convert
     * @return
     *  Converted value
     */
    public static double inchesToMeters(double inches){
        return inches * 0.0254;
    }

    /**
     * @param meters
     *  Value to convert
     * @return
     *  Converted value
     */
    public static double metersToInches(double meters){
        return meters * 39.3701;
    }
}