package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final String NT_APPLY_PID_BUTTON = "Tuning/Shooter/ApplyPIDButton";

    /* ================== */
    /* FLYWHEEL CONSTANTS */
    /* ================== */
    
    public static double SHOOTER_GEAR_RATIO = 1.0;
    public static double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static int DEFAULT_SPEED_MPS = 15;
    public static final double SHOOTER_SPEED_TOLERANCE = 0.05; // in m/s

    static class PID_CONFIG {
        static final double STATIC = 0.0;
        static final double VELOCITY = 0.123;
        static final double ACCELERATION = 0.0;        
        static final double PROPORTIONAL = 0.50;
        static final double INTEGRAL = 0.0;
        static final double DERIVATIVE = 0.0;
    }

    /** HOOD CONSTANTS */
    public static double HOOD_GEAR_RATIO = 20.0;
    public static double HOOD_STARTING_POSITION = 0.0;
    public static double HOOD_TARGET_POSITION = -0.4;
    public static double HOOD_MAX_POSITION = -1.542236328125;
    public static double HOOD_MIN_POSITION = 0.0;
    public static final double HOOD_POSITION_TOLERANCE = 0.01;
    
    static class HOOD_PID_CONFIG {
        static final double PROPORTIONAL = 100.0;
        static final double DERIVATIVE = 0.5;
    }

    /** SHOOT FROM ANYWHERE DATA */
    public static final InterpolatingDoubleTreeMap SHOOT_VELOCITY_MAP = new InterpolatingDoubleTreeMap(); // map of distance (in) -> velocity (m/s)
    public static final InterpolatingDoubleTreeMap SHOOT_HOOD_POSITION_MAP = new InterpolatingDoubleTreeMap(); // map of distance (in) -> hood rotations (0.0 - -1.5)

    static {
        // distance from the front bumper of the robot to the hub in inches, velocity in m/s
        SHOOT_VELOCITY_MAP.put(36.257, 14.75);
        SHOOT_VELOCITY_MAP.put(78.0, 15.8);
        SHOOT_VELOCITY_MAP.put(156.67, 19.116);
        SHOOT_VELOCITY_MAP.put(128.65, 17.85);
        SHOOT_VELOCITY_MAP.put(90.0, 16.517);
        SHOOT_VELOCITY_MAP.put(55.0, 15.0);
        
        // distance to the hub in inches, position in mechanism rotations
        SHOOT_HOOD_POSITION_MAP.put(36.257, -0.375);
        SHOOT_HOOD_POSITION_MAP.put(78.0, -0.644);
        SHOOT_HOOD_POSITION_MAP.put(156.67, -1.152);
        SHOOT_HOOD_POSITION_MAP.put(128.65, -0.985);
        SHOOT_HOOD_POSITION_MAP.put(90.0, -0.725);
        SHOOT_HOOD_POSITION_MAP.put(55.0, -0.49);
    }

    public static final InterpolatingDoubleTreeMap PASS_VELOCITY_MAP = new InterpolatingDoubleTreeMap();

    static {
        PASS_VELOCITY_MAP.put(345.851142, 25.0);
        PASS_VELOCITY_MAP.put(181.851142, 15.96);
        PASS_VELOCITY_MAP.put(109.851142, 12.0);
    }
}