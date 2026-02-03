package frc.robot.subsystems.drivetrain;

import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

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
}
