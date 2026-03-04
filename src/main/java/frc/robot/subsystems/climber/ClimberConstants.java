package frc.robot.subsystems.climber;

public class ClimberConstants {

    public static final double GEAR_RATIO = 20.0; 

    // Up is negative, down is positive
    public static double DEFAULT_DUTY_CYCLE = 0.2;

    // we seed the starting position to this, so 0.0 is 'fully down' in match cases.
    public static double START_POSITION = 0.0;

    public static double DEFAULT_PEAK_POSITION = -1.5;
    public static double DEFAULT_FINAL_POSITION = 0;

    public static double kP = 3.2;
    public static double kI = 0.0;
    public static double kD = 0.05;
    public static double POSITION_TOLERANCE = 0.02; // in rotations of the mechanism
}
