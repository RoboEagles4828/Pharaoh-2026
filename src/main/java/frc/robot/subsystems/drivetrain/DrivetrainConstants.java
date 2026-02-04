package frc.robot.subsystems.drivetrain;

import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;

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
}
