package frc.robot;

public class ElementMath {
    public static final double kEpsilon = 1e-9;
	static double prevArrVal;

	// Higher sensitivity on joystick
	public static double squareInput(double input) {
		double normalize = input < 0.0 ? -1.0 : 1.0;
		return Math.pow(input, 2) * normalize;
	}

	// Higher sensitivity on joystick
	public static double cubeInput(double input) {
		return Math.pow(input, 3);
	}

	// Raise input to the fourth power
	public static double biquadrateInput(double input) {
		return Math.pow(input, 4) * input/Math.abs(input);
	}

	// Deadband for TeleOp Drive
	public static double handleDeadband(double val, double deadband){
		val = (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;

		if (val != 0)
			val = Math.signum(val) * ((Math.abs(val) - deadband) / (1.0 - deadband));

		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}
	
	public static boolean isWithinDeadband(double val, double deadband){
		val = (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
		boolean isTrue = false;

		if (val != 0)
			val = Math.signum(val) * ((Math.abs(val) - deadband) / (1.0 - deadband));

		if(Math.abs(val) > Math.abs(deadband)){
			isTrue = true;
		}else{
			isTrue = false;
		}
		return isTrue;
	}

	public static boolean epsilonEquals(double a, double b, double epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

	public static boolean epsilonEquals(double a, double b) {
		return epsilonEquals(a, b, kEpsilon);
	}

	// limits input to be within a certain range
	public static double limit(double v, double maxMagnitude) {
		return limit(v, -maxMagnitude, maxMagnitude);
	}

	public static double limit(double v, double min, double max) {
		return Math.min(max, Math.max(min, v));
	}

	public static boolean inRange(double v, double min, double max) {
		return v > min && v < max;
	}

	public static double interpolate(double a, double b, double x) {
		x = limit(x, 0.0, 1.0);
		return a + (b - a) * x;
	}

}

