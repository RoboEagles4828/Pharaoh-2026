package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final String NT_APPLY_PID_BUTTON = "Tuning/Shooter/ApplyPIDButton";

    /** FLYWHEEL CONSTANTS */
    public static final String NT_ACTUAL_SPEED_MPS_ONE = "Tuning/Shooter/ActualShooterSpeedMPSOne";
    public static final String NT_ACTUAL_SPEED_MPS_TWO = "Tuning/Shooter/ActualShooterSpeedMPSTwo"; 
    public static final String NT_ACTUAL_SPEED_MPS_THREE = "Tuning/Shooter/ActualShooterSpeedMPSThree";
    public static final String NT_ACTUAL_KICKER_SPEED_MPS = "Tuning/Shooter/ActualKickerSpeedMPS";
    public static final String NT_ACTUAL_HOOD_POSITION = "Tuning/Shooter/ActualHoodPosition";

    public static final String NT_SHOOTER_P_VALUE = "Tuning/Shooter/ShooterPValue";
    public static final String NT_SHOOTER_V_VALUE = "Tuning/Shooter/ShooterVValue";

    public static double SHOOTER_GEAR_RATIO = 1.0;
    public static double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static int DEFAULT_SPEED_MPS = 10;

    static class PID_CONFIG {
        static final double GRAVITY = 0.0;
        static final double STATIC = 0.0;
        static final double VELOCITY = 0.123;
        static final double ACCELERATION = 0.0;        
        static final double PROPORTIONAL = 0.2;
        static final double INTEGRAL = 0.0;
        static final double DERIVATIVE = 0.0;
    }

    /** KICKER CONSTANTS */
    public static final String NT_KICKER_TARGET_SPEED_MPS = "Tuning/Shooter/TargetKickerSpeedMPS";
    public static final String NT_KICKER_P_VALUE = "Tuning/Shooter/KickerPValue";
    public static final String NT_KICKER_V_VALUE = "Tuning/Shooter/KickerVValue";
    public static double KICKER_GEAR_RATIO = 4.0;
    public static double KICKER_WHEEL_DIAMETER = Units.inchesToMeters(2.25);
    public static int DEFAULT_KICKER_SPEED_MPS = 5;
    
    static class KICKER_PID_CONFIG {
        static final double VELOCITY = 0.39;
        static final double PROPORTIONAL = 0.1;
    }

    /** HOOD CONSTANTS */
    
    public static final String NT_TARGET_SPEED_MPS = "Tuning/Shooter/TargetShooterSpeedMPS";
    public static final String NT_TARGET_HOOD_POSITION = "Tuning/Shooter/TargetHoodPosition";
    public static final String NT_HOOD_P_VALUE = "Tuning/Shooter/HoodPValue";
    public static final String NT_HOOD_D_VALUE = "Tuning/Shooter/HoodDValue";
    public static double HOOD_GEAR_RATIO = 20.0;
    public static double HOOD_STARTING_POSITION = 0.0;
    public static double HOOD_TARGET_POSITION = -0.75;
    public static double HOOD_MAX_POSITION = -1.5;
    public static double HOOD_MIN_POSITION = 0.0;
    static class HOOD_PID_CONFIG {
        static final double PROPORTIONAL = 110.0;
        static final double DERIVATIVE = 1.0;
    }
}