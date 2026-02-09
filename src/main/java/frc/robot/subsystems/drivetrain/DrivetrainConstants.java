package frc.robot.subsystems.drivetrain;

import frc.robot.util.TunableNumber;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    public static final double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double DEADBAND = 0.1;
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final double ROTATIONAL_DEADBAND = 0.1;
    public static final double MAX_AUTOALIGN_TOWER_DISTANCE = 4.0;

    public static final double TOWER_ALIGN_STEP1_X = 1.107;
    public static final double TOWER_ALIGN_STEP1_Y = 4.675;
    public static final double TOWER_ALIGN_STEP1_THETA = 0; // degrees
    public static final double TOWER_ALIGN_STEP2_X = 1.775;
    public static final double TOWER_ALIGN_STEP2_Y = 4.675;
    public static final double TOWER_ALIGN_STEP2_THETA = 0; // degrees

    static class PathPlannerConstraints {
        public static final PathConstraints SAFE = new PathConstraints(
            0.5,  // max velocity (m/s)
            0.5,  // max accel (m/s^2)
            1.0,  // max angular vel (rad/s)
            1.0   // max angular accel
        );
    }

    // PID Auto Align Constants
    public static class PIDAutoAlignConstants {
        public static final PIDConstants X_PID_CONSTANTS = new PIDConstants(
            1.0, // kP
            0.0, // kI
            0.0  // kD
        );
        public static final PIDConstants Y_PID_CONSTANTS = new PIDConstants(
            1.0, // kP
            0.0, // kI
            0.0  // kD
        );
        public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(
            0.1, // kP
            0.0, // kI
            0.0  // kD
        );
        
        public static final double POSITION_ERROR_DERIVATIVE_TOLERANCE = 0.0; // m/s
        public static final double ROTATION_ERROR_DERIVATIVE_TOLERANCE = 0.0; // rad/s

        public static final double X_I_ZONE = Units.inchesToMeters(6.0); // meters
        public static final double X_TOLERANCE = Units.inchesToMeters(1.0);

        public static final double Y_I_ZONE = Units.inchesToMeters(6.0); // meters
        public static final double Y_TOLERANCE = Units.inchesToMeters(1.0);

        public static final double ROTATION_I_ZONE = Math.toRadians(10); // radians
        public static final double ROTATION_TOLERANCE = Math.toRadians(2); // radians

        public static final double MAX_SPEED = 0.5; // max speed during auto-align (m/s)
        public static final double MAX_ANGULAR_SPEED = 0.5; // max angular speed during auto-align (rad/s)

        public static final double TIMEOUT = 5.0; // seconds
    }
}
