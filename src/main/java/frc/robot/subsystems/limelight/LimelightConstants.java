package frc.robot.subsystems.limelight;

public class LimelightConstants {
    public static String LIMELIGHT_ONE_NAME = "limelight-one";
    public static String LIMELIGHT_TWO_NAME = "limelight-two";

    final static double POSE_AMBIGUITY_THRESHOLD = 0.7; // < Reject the pose if ambiguity is above this.
    final static double POSE_DISTANCE_THRESHOLD = 15.0; // < Reject the pose if distance is above this (meters).

    final static double STD_MIN_XY = 0.1; // meters
    final static double STD_MAX_XY = 2.0; // meters
    final static double STD_MIN_THETA = Math.toRadians(5); // radians
    final static double STD_MAX_THETA = Math.toRadians(45); // radians
}
