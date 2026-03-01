package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;

public class DrivetrainConstants {
    /** Teloperated max speed of the robot in meters per second */
    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double DEADBAND = 0.1;
    /** Teleoperated max angular valocity of the robot in radians per second */
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final double ROTATIONAL_DEADBAND = 0.1;
    public static final double MAX_AUTOALIGN_TOWER_DISTANCE = 3.0;

    static class LockOnDriveConstraints {
        public static final double kP = 12.0; 
        public static final double kI = 0.0;
        public static final double kD = 0.5;
        public static final double MAX_ROTATIONAL_VELOCITY = 12.0; // rad/s
        public static final double MAX_ROTATIONAL_ACCELERATION = 18.0; // rad/s²
        public static final double AIM_TOLERANCE_DEGREES = 1.5; // degrees
    }

    static class PathPlannerConstraints {

        public static final double TOWER_ALIGN_STEP1_X_LEFT = 1.607;
        public static final double TOWER_ALIGN_STEP1_Y_LEFT = 4.675;
        public static final double TOWER_ALIGN_STEP1_THETA_LEFT = 180; // degrees
        public static final double TOWER_ALIGN_STEP2_X_LEFT = 1.1;
        public static final double TOWER_ALIGN_STEP2_Y_LEFT = 2.9;
        public static final double TOWER_ALIGN_STEP2_THETA_LEFT = -180; // degrees

        public static final double TOWER_ALIGN_STEP1_X_RIGHT = 1.607;
        public static final double TOWER_ALIGN_STEP1_Y_RIGHT = 2.730;
        public static final double TOWER_ALIGN_STEP1_THETA_RIGHT = 0; // degrees
        public static final double TOWER_ALIGN_STEP2_X_RIGHT = 1.107;
        public static final double TOWER_ALIGN_STEP2_Y_RIGHT = 4.675;
        public static final double TOWER_ALIGN_STEP2_THETA_RIGHT = 0; // degrees

        public static final PathConstraints SAFE = new PathConstraints(
            0.5,  // max velocity (m/s)
            0.5,  // max accel (m/s^2)
            1.0,  // max angular vel (rad/s)
            1.0   // max angular accel
        );

        public static final PathConstraints FAST = new PathConstraints(
            3.0,  // max velocity (m/s)
            2.0,  // max accel (m/s^2)
            4.0,  // max angular vel (rad/s)
            3.0   // max angular accel
        );
    }
}
