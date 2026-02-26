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
    public static double HOOD_MAX_POSITION = -1.5;
    public static double HOOD_MIN_POSITION = 0.0;
    static class HOOD_PID_CONFIG {
        static final double PROPORTIONAL = 110.0;
        static final double DERIVATIVE = 1.0;
    }

    /** SHOOT FROM ANYWHERE DATA */
    public static final InterpolatingDoubleTreeMap SHOOT_VELOCITY_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap SHOOT_HOOD_POSITION_MAP = new InterpolatingDoubleTreeMap();

    static {

        // distance from the front bumper of the robot to the hub in feet, velocity in m/s
        SHOOT_VELOCITY_MAP.put(0.0, 0.0);
        SHOOT_VELOCITY_MAP.put(1.0, 0.0);
        SHOOT_VELOCITY_MAP.put(2.0, 0.0);
        SHOOT_VELOCITY_MAP.put(3.0, 0.0);
        SHOOT_VELOCITY_MAP.put(4.0, 0.0);
        SHOOT_VELOCITY_MAP.put(5.0, 0.0);
        SHOOT_VELOCITY_MAP.put(6.0, 0.0);
        SHOOT_VELOCITY_MAP.put(7.0, 0.0);
        SHOOT_VELOCITY_MAP.put(8.0, 0.0);
        SHOOT_VELOCITY_MAP.put(9.0, 0.0);
        SHOOT_VELOCITY_MAP.put(10.0, 0.0);
        SHOOT_VELOCITY_MAP.put(11.0, 0.0);
        SHOOT_VELOCITY_MAP.put(12.0, 0.0);
        SHOOT_VELOCITY_MAP.put(13.0, 0.0);
        SHOOT_VELOCITY_MAP.put(14.0, 0.0);
        SHOOT_VELOCITY_MAP.put(15.0, 0.0);
        SHOOT_VELOCITY_MAP.put(16.0, 0.0);
        SHOOT_VELOCITY_MAP.put(17.0, 0.0);
        SHOOT_VELOCITY_MAP.put(18.0, 0.0);
        SHOOT_VELOCITY_MAP.put(19.0, 0.0);

        // distance to the hub in feet, position in mechanism rotations
        SHOOT_HOOD_POSITION_MAP.put(0.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(1.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(2.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(3.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(4.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(5.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(6.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(7.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(8.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(9.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(10.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(11.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(12.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(13.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(14.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(15.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(16.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(17.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(18.0, 0.0);
        SHOOT_HOOD_POSITION_MAP.put(19.0, 0.0);
    }

    public static final InterpolatingDoubleTreeMap PASS_VELOCITY_MAP = new InterpolatingDoubleTreeMap();

    static {
        PASS_VELOCITY_MAP.put(0.0, 0.0);
        PASS_VELOCITY_MAP.put(5.0, 0.0);
        PASS_VELOCITY_MAP.put(10.0, 0.0);
        PASS_VELOCITY_MAP.put(15.0, 0.0);
        PASS_VELOCITY_MAP.put(20.0, 0.0);
        PASS_VELOCITY_MAP.put(25.0, 0.0);
        PASS_VELOCITY_MAP.put(30.0, 0.0);
        PASS_VELOCITY_MAP.put(35.0, 0.0);
        PASS_VELOCITY_MAP.put(40.0, 0.0);
        PASS_VELOCITY_MAP.put(45.0, 0.0);
        PASS_VELOCITY_MAP.put(50.0, 0.0);
    }

    public static final double SECONDS_OF_DATA_TO_AVERAGE = 0.1; // how many seconds of data to use for the moving average filter in the launch calculator
    

    public static final Translation2d CENTER_HUB = 
        new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(317.69 / 2));
    
    public static final Translation2d TOP_PASS_POINT =
        new Translation2d(Units.inchesToMeters(156.61 / 2), Units.inchesToMeters(317.69 * 0.75));

     public static final Translation2d BOTTOM_PASS_POINT = 
        new Translation2d(Units.inchesToMeters(156.61 / 2), Units.inchesToMeters(317.69 / 4));

}