package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final String NT_APPLY_PID_BUTTON = "Tuning/Shooter/ApplyPIDButton";

    /** FLYWHEEL CONSTANTS */
    public static final String NT_ACTUAL_SPEED_MPS_ONE = "Tuning/Shooter/ActualShooterSpeedMPSOne";
    public static final String NT_ACTUAL_SPEED_MPS_TWO = "Tuning/Shooter/ActualShooterSpeedMPSTwo"; 
    public static final String NT_ACTUAL_SPEED_MPS_THREE = "Tuning/Shooter/ActualShooterSpeedMPSThree";
    public static final String NT_ACTUAL_HOOD_POSITION = "Tuning/Shooter/ActualHoodPosition";

    public static final String NT_SHOOTER_P_VALUE = "Tuning/Shooter/ShooterPValue";
    public static final String NT_SHOOTER_V_VALUE = "Tuning/Shooter/ShooterVValue";

    public static double SHOOTER_GEAR_RATIO = 1.0;
    public static double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static int DEFAULT_SPEED_MPS = 15;

    static class PID_CONFIG {
        static final double GRAVITY = 0.0;
        static final double STATIC = 0.0;
        static final double VELOCITY = 0.123;
        static final double ACCELERATION = 0.0;        
        static final double PROPORTIONAL = 0.33;
        static final double INTEGRAL = 0.0;
        static final double DERIVATIVE = 0.0;
    }

    /** HOOD CONSTANTS */
    public static final String NT_TARGET_SPEED_MPS = "Tuning/Shooter/TargetShooterSpeedMPS";
    public static final String NT_TARGET_HOOD_POSITION = "Tuning/Shooter/TargetHoodPosition";
    public static final String NT_HOOD_P_VALUE = "Tuning/Shooter/HoodPValue";
    public static final String NT_HOOD_D_VALUE = "Tuning/Shooter/HoodDValue";
    public static double HOOD_GEAR_RATIO = 20.0;
    public static double HOOD_STARTING_POSITION = 0.0;
    public static double HOOD_TARGET_POSITION = -0.4;
    public static double HOOD_MAX_POSITION = -1.5;
    public static double HOOD_MIN_POSITION = 0.0;
    static class HOOD_PID_CONFIG {
        static final double PROPORTIONAL = 110.0;
        static final double DERIVATIVE = 1.0;
    }

    /** SHOOT FROM ANYWHERE DATA */
    public static final ShooterParameters[] SHOOT_LOOKUP_TABLE = new ShooterParameters[20]; 
    public static final int SHOOT_TABLE_INTERVAL = 1; // 1 ft between each entry

    static {
        SHOOT_LOOKUP_TABLE[0] = new ShooterParameters(0, 6, 0.05);
        SHOOT_LOOKUP_TABLE[1] = new ShooterParameters(1, 6, 0.05);
        SHOOT_LOOKUP_TABLE[2] = new ShooterParameters(2, 6, 0.1);
        SHOOT_LOOKUP_TABLE[3] = new ShooterParameters(3, 6, 0.15);
        SHOOT_LOOKUP_TABLE[4] = new ShooterParameters(4, 6, 0.2);
        SHOOT_LOOKUP_TABLE[5] = new ShooterParameters(5, 6, 0.25);
        SHOOT_LOOKUP_TABLE[6] = new ShooterParameters(6, 6, 0.3);
        SHOOT_LOOKUP_TABLE[7] = new ShooterParameters(7, 6, 0.35);
        SHOOT_LOOKUP_TABLE[8] = new ShooterParameters(8, 6, 0.4);
        SHOOT_LOOKUP_TABLE[9] = new ShooterParameters(9, 6, 0.45);
        SHOOT_LOOKUP_TABLE[10] = new ShooterParameters(10, 6, 0.5);
        SHOOT_LOOKUP_TABLE[11] = new ShooterParameters(11, 6, 0.5);
        SHOOT_LOOKUP_TABLE[12] = new ShooterParameters(12, 6, 0.5);
        SHOOT_LOOKUP_TABLE[13] = new ShooterParameters(13, 6, 0.5);
        SHOOT_LOOKUP_TABLE[14] = new ShooterParameters(14, 6, 0.5);
        SHOOT_LOOKUP_TABLE[15] = new ShooterParameters(15, 6, 0.5);
        SHOOT_LOOKUP_TABLE[16] = new ShooterParameters(16, 6, 0.5);
        SHOOT_LOOKUP_TABLE[17] = new ShooterParameters(17, 6, 0.5);
        SHOOT_LOOKUP_TABLE[18] = new ShooterParameters(18, 6, 0.5);
        SHOOT_LOOKUP_TABLE[19] = new ShooterParameters(19, 6, 0.5);
    }

    public static final ShooterParameters[] PASS_LOOKUP_TABLE = new ShooterParameters[11];
    public static final int PASS_TABLE_INTERVAL = 5; // 5ft per entry
    static {
        PASS_LOOKUP_TABLE[0] = new ShooterParameters(0, 6, 0.05);
        PASS_LOOKUP_TABLE[1] = new ShooterParameters(5, 6, 0.05);
        PASS_LOOKUP_TABLE[2] = new ShooterParameters(10, 6, 0.05);
        PASS_LOOKUP_TABLE[3] = new ShooterParameters(15, 6, 0.05);
        PASS_LOOKUP_TABLE[4] = new ShooterParameters(20, 6, 0.05);
        PASS_LOOKUP_TABLE[5] = new ShooterParameters(25, 6, 0.05);
        PASS_LOOKUP_TABLE[6] = new ShooterParameters(30, 6, 0.05);
        PASS_LOOKUP_TABLE[7] = new ShooterParameters(35, 6, 0.05);
        PASS_LOOKUP_TABLE[8] = new ShooterParameters(40, 6, 0.05);
        PASS_LOOKUP_TABLE[9] = new ShooterParameters(45, 6, 0.05);
        PASS_LOOKUP_TABLE[9] = new ShooterParameters(50, 6, 0.05);
    }
}