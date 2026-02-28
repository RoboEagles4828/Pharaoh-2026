package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Util4828;

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
    public static double HOOD_MAX_POSITION = -1.542236328125;
    public static double HOOD_MIN_POSITION = 0.0;
    static class HOOD_PID_CONFIG {
        static final double PROPORTIONAL = 110.0;
        static final double DERIVATIVE = 1.0;
    }

    /** SHOOT FROM ANYWHERE DATA */
    public static final InterpolatingDoubleTreeMap SHOOT_VELOCITY_MAP = new InterpolatingDoubleTreeMap(); // map of distance (in) -> velocity (m/s)
    public static final InterpolatingDoubleTreeMap SHOOT_HOOD_POSITION_MAP = new InterpolatingDoubleTreeMap(); // map of distance (in) -> hood rotations (0.0 - -1.5)

    static {
        // distance from the front bumper of the robot to the hub in inches, velocity in m/s
        SHOOT_VELOCITY_MAP.put(205.851142, 21.0);
        SHOOT_VELOCITY_MAP.put(193.851142, 20.57);
        SHOOT_VELOCITY_MAP.put(181.851142, 20.13);
        SHOOT_VELOCITY_MAP.put(169.851142, 19.7);
        SHOOT_VELOCITY_MAP.put(157.851142, 19.27);
        SHOOT_VELOCITY_MAP.put(145.851142, 18.83);
        SHOOT_VELOCITY_MAP.put(133.851142, 18.4);
        SHOOT_VELOCITY_MAP.put(109.851142, 17.43);
        SHOOT_VELOCITY_MAP.put(85.851142, 16.64);
        SHOOT_VELOCITY_MAP.put(39.851142, 15.0);
        
        // distance to the hub in feet, position in mechanism rotations
        SHOOT_HOOD_POSITION_MAP.put(205.851142, -1.25);
        SHOOT_HOOD_POSITION_MAP.put(193.851142, -1.181);
        SHOOT_HOOD_POSITION_MAP.put(181.851142, -1.112);
        SHOOT_HOOD_POSITION_MAP.put(169.851142, -1.104);
        SHOOT_HOOD_POSITION_MAP.put(157.851142, -0.99);
        SHOOT_HOOD_POSITION_MAP.put(145.851142, -0.92);
        SHOOT_HOOD_POSITION_MAP.put(133.851142, -0.85);
        SHOOT_HOOD_POSITION_MAP.put(109.851142, -0.711);
        SHOOT_HOOD_POSITION_MAP.put(85.851142, -0.53);
        SHOOT_HOOD_POSITION_MAP.put(39.851142, -0.3);
    }

    public static final InterpolatingDoubleTreeMap PASS_VELOCITY_MAP = new InterpolatingDoubleTreeMap();

    static {
        PASS_VELOCITY_MAP.put(345.851142, 25.0);
        PASS_VELOCITY_MAP.put(181.851142, 15.96);
        PASS_VELOCITY_MAP.put(109.851142, 12.0);
    }
}