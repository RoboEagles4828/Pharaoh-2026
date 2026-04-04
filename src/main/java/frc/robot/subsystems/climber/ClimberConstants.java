package frc.robot.subsystems.climber;

public class ClimberConstants {

    public static final double GEAR_RATIO = 20.0; 

    // Up is negative, down is positive
    public static final double DEFAULT_DUTY_CYCLE = 0.2;

    // we seed the starting position to this, so 0.0 is 'fully down' in match cases.
    public static final double START_POSITION = 0.0;
    public static final double CLIMB_POSITION = -3.802490234375;

    public static final double kP = 9.0;
    public static final double kI = 0.0;
    public static final double kD = 0.03;
    public static final double POSITION_TOLERANCE = 0.02; // in rotations of the mechanism

    public static final double CURRENT_LIMIT = 40.0; // in amps
}
