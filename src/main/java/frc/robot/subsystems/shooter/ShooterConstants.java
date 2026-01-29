package frc.robot.subsystems.shooter;

public class ShooterConstants {
    public static int GEAR_RATIO = 1;
    public static int DEFAULT_SHOOTING_SPEED_RPS = 30;
    public static final String NT_SHOOTING_SPEED_KEY = "Tuning/Shooter/SpeedRPS";

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