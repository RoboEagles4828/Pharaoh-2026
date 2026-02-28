package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LaunchCalculator {
    private static LaunchCalculator instance;

    public static LaunchCalculator getInstance() {
        if (instance == null) {
            instance = new LaunchCalculator();
        }
        return instance;
    }

    public record LaunchParameters(double velocityMPS, double hoodPosition) {}

    // maps for scoring
    private static final InterpolatingDoubleTreeMap launchVelocityMap = ShooterConstants.SHOOT_VELOCITY_MAP;
    private static final InterpolatingDoubleTreeMap launchHoodPositionMap = ShooterConstants.SHOOT_HOOD_POSITION_MAP;

    // maps for passing
    private static final InterpolatingDoubleTreeMap passVelocityMap = ShooterConstants.PASS_VELOCITY_MAP;

    public LaunchParameters getParameters(Pose2d robotPose, boolean isScoring) {
        boolean targetTop = robotPose.getY() > ShooterConstants.CENTER_HUB.getY();

        Translation2d targetPos = isScoring ? ShooterConstants.CENTER_HUB : 
            targetTop ? ShooterConstants.TOP_PASS_POINT : ShooterConstants.BOTTOM_PASS_POINT;
        
        // Calculating parameters to shoot from anywhere
        double distanceToTarget = robotPose.getTranslation().getDistance(targetPos);
        double targetVelocity = isScoring ? launchVelocityMap.get(distanceToTarget) : passVelocityMap.get(distanceToTarget);
        double targetHoodPosition = isScoring ? launchHoodPositionMap.get(distanceToTarget) : ShooterConstants.HOOD_MAX_POSITION;

        return new LaunchParameters(targetVelocity, targetHoodPosition);
    }

}
