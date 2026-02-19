package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final String NT_APPLY_PID_BUTTON = "Tuning/Shooter/ApplyPIDButton";

    public static final String NT_KICKER_TARGET_SPEED_MPS = "Tuning/Shooter/KickerSpeedMPS";
    public static final String NT_TARGET_SPEED_MPS = "Tuning/Shooter/ShooterSpeedMPS";
    public static final String NT_TARGET_HOOD_POSITION = "Tuning/Shooter/HoodPosition";
    public static final String NT_HOOD_P_VALUE = "Tuning/Shooter/HoodPValue";
    
    public static final String NT_ACTUAL_SPEED_MPS = "Tuning/Shooter/ActualShooterSpeedMPS";
    public static final String NT_ACTUAL_KICKER_SPEED_MPS = "Tuning/Shooter/ActualKickerSpeedMPS";
    public static final String NT_ACTUAL_HOOD_POSITION = "Tuning/Shooter/ActualHoodPosition";

    public static double SHOOTER_GEAR_RATIO = 1.0;
    public static double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static int DEFAULT_SPEED_MPS = 20;

    public static double KICKER_GEAR_RATIO = 4.0;
    public static double KICKER_WHEEL_DIAMETER = Units.inchesToMeters(2.25);
    public static int DEFAULT_KICKER_SPEED_MPS = 5;

    public static double HOOD_GEAR_RATIO = 20.0;
    public static double HOOD_STARTING_POSITION = 0.0;
    static class HOOD_PID_CONFIG {
        static final double PROPORTIONAL = 1.0;
        static final double DERIVATIVE = 0.0;
    }

    public static final String NT_SHOOTER_P_VALUE = "Tuning/Shooter/ShooterPValue";
    public static final String NT_SHOOTER_V_VALUE = "Tuning/Shooter/ShooterVValue";

    static class PID_CONFIG {
        static final double GRAVITY = 0.0;
        static final double STATIC = 0.0;
        static final double VELOCITY = 0.12;
        static final double ACCELERATION = 0.0;        
        static final double PROPORTIONAL = 0.1;
        static final double INTEGRAL = 0.0;
        static final double DERIVATIVE = 0.0;
    }
}