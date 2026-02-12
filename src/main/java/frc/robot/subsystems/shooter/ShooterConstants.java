package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final String NT_USE_DASHBOARD_VALUES = "Tuning/Shooter/UseDashboardValues";
    public static final String NT_OVERRIDE_TARGET_SPEED = "Tuning/Shooter/OverrideTargetSpeed";
    public static final String NT_OVERRIDE_TARGET_HOOD = "Tuning/Shooter/OverrideTargetHood";
    
    public static final String NT_ACTUAL_SPEED_MPS = "Tuning/Shooter/ActualSpeedMPS";
    public static final String NT_TARGET_SPEED_MPS = "Tuning/Shooter/SpeedMPS";
    public static final String NT_TARGET_HOOD = "Tuning/Shooter/Hood";
    public static final String NT_ACTUAL_KICKER_SPEED_MPS = "Tuning/Shooter/ActualKickerSpeedMPS";
    public static final String NT_KICKER_TARGET_SPEED_MPS = "Tuning/Shooter/KickerSpeedMPS";

    public static double SHOOTER_GEAR_RATIO = 1.0;
    public static double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static int DEFAULT_SPEED_MPS = 20;

    public static double KICKER_GEAR_RATIO = 4.0;
    public static double KICKER_WHEEL_DIAMETER = Units.inchesToMeters(2.25);
    public static int DEFAULT_KICKER_SPEED_MPS = 5;

    static class PID_CONFIG {
        static final double GRAVITY = 0.0;
        static final double STATIC = 0.0;
        static final double VELOCITY = 0.12;
        static final double ACCELERATION = 0.0;        
        static final double PROPORTIONAL = 0.1;
        static final double INTEGRAL = 0.0;
        static final double DERIVATIVE = 0.0;
    }

    public static final ShooterParameters[] SHOOTER_LOOKUP_TABLE = 
        new ShooterParameters[200]; 
        
    static{

        SHOOTER_LOOKUP_TABLE[1] = new ShooterParameters(0.5, 6, 0.05);
        SHOOTER_LOOKUP_TABLE[2] = new ShooterParameters(1, 6, 0.1);
        SHOOTER_LOOKUP_TABLE[3] = new ShooterParameters(1.5, 6, 0.15);
        SHOOTER_LOOKUP_TABLE[4] = new ShooterParameters(2, 6, 0.2);
        SHOOTER_LOOKUP_TABLE[5] = new ShooterParameters(2.5, 6, 0.25);
        SHOOTER_LOOKUP_TABLE[6] = new ShooterParameters(3, 6, 0.3);
        SHOOTER_LOOKUP_TABLE[7] = new ShooterParameters(3.5, 6, 0.35);
        SHOOTER_LOOKUP_TABLE[8] = new ShooterParameters(4, 6, 0.4);
        SHOOTER_LOOKUP_TABLE[9] = new ShooterParameters(4.5, 6, 0.45);
        SHOOTER_LOOKUP_TABLE[10] = new ShooterParameters(5, 6, 0.5);
    }
}
