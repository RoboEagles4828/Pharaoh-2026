package frc.robot.subsystems.vision;

public class VisionConstants {
    public static String LIMELIGHT_FORWARD_NAME = "limelight-forward";
    public static int LIMELIGHT_FORWARD_USB_PORT = 0;
    public static String LIMELIGHT_UP_NAME = "limelight-up";
    public static int LIMELIGHT_UP_USB_PORT = 1;

    final static double POSE_AMBIGUITY_THRESHOLD = 0.7; // < Reject the pose if ambiguity is above this.
    final static double POSE_DISTANCE_THRESHOLD = 15.0; // < Reject the pose if distance is above this (meters).

    final static double STD_MIN_XY = 0.1; // meters
    final static double STD_MAX_XY = 2.0; // meters
    final static double STD_MIN_THETA = Math.toRadians(5); // radians
    final static double STD_MAX_THETA = Math.toRadians(45); // radians

    public static final String NT_USE_VISION_TOGGLE = "LL Use Vision";
}
