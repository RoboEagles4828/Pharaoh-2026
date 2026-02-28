package frc.robot.subsystems.kicker;

import edu.wpi.first.math.util.Units;

public class KickerConstants {
    /** KICKER CONSTANTS */
    public static final String NT_ACTUAL_KICKER_SPEED_MPS = "Tuning/Kicker/ActualKickerSpeedMPS";
    public static final String NT_KICKER_TARGET_SPEED_MPS = "Tuning/Kicker/TargetKickerSpeedMPS";
    public static final String NT_KICKER_P_VALUE = "Tuning/Kicker/KickerPValue";
    public static final String NT_KICKER_V_VALUE = "Tuning/Kicker/KickerVValue";
    
    public static double KICKER_GEAR_RATIO = 4.0;
    public static double KICKER_WHEEL_DIAMETER = Units.inchesToMeters(2.25);
    public static int DEFAULT_KICKER_SPEED_MPS = 5;
    
    static class KICKER_PID_CONFIG {
        static final double VELOCITY = 0.39;
        static final double PROPORTIONAL = 0.1;
    }
}
