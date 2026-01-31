package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final String NT_ACTUAL_SPEED_MPS = "Tuning/Shooter/ActualSpeedMPS";
    public static final String NT_TARGET_SPEED_MPS = "Tuning/Shooter/SpeedMPS";

    public static int GEAR_RATIO = 1;
    public static double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static int DEFAULT_SPEED_MPS = 20;

    static class PID_CONFIG {
        static final double GRAVITY = 0.0;
        static final double STATIC = 0.0;
        static final double VELOCITY = 0.12;
        static final double ACCELERATION = 0.0;
        static final double INTEGRAL = 0.0;
        static final double PROPORTIONAL = 0.1;
        static final double DERIVATIVE = 0.0;
    }
}