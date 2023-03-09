package frc.robot;

/**
 * General math equations we use based around units or unit conversions
 */

//TODO: run unit testing on unit conversions
public class ElementUnits {
    public static double inchesToRotations(double inches, double circumference, double gearRatio) {
        return inches / ((circumference * gearRatio));

    }
    public static double rotationsToInches(double rotations, double circumference, double gearRatio) {
        return rotations * ((circumference * gearRatio));

    }

    public static double radiansPerSecondToTicksPer100ms(double rad_s, double encoderPPR) {
        return rad_s / (Math.PI * 2.0) * encoderPPR / 10.0;
    }

    public static double rotationsToTicks(double rotations, double ppr){
        return rotations * ppr;
    }

    public static double ticksToRotations(double ticks, double ppr){
        return ticks / ppr;
    }

    public static double tickPer100msToRPM(double ticks, double ppr){
        return ticksToRotations(ticks, ppr) * 600;
    }

    public static double tickPer100msToScaledRPM(double ticks, double ppr, double gearRatio){
        return scaleRPM(ticksToRotations(ticks * 600, ppr), gearRatio);
    }

    public static double rpmToTicksPer100ms(double rpm, double ppr){
        return rotationsToTicks(rpm / 600, ppr);
    }

    // functions for converting between rpm of a wheel and the gear ratio to the motor
    public static double scaleRPM(double initRPM, double gearRatio){
        return initRPM / gearRatio;
    }

    public static double unscaleRPM(double finRPM, double gearRatio){
        return finRPM * gearRatio;
    }
}