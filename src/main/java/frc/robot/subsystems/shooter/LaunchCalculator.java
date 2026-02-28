package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.util.Util4828;
import edu.wpi.first.wpilibj.DriverStation;

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

    public LaunchParameters getParameters(Pose2d robotPose) {
        // Are we scoring or passing? If we're in our alliance zone, we're trying to score.
        boolean isScoring = Util4828.isInAllianceZone(robotPose);

        // Get the target position. This will either be the hub center for our alliance,
        // or the top/bottom pass position. It depends on the robot's pose.
        // If we are in our alliance zone - it will be our hub.
        // If we are not - it will be a passing position. Top pass if we are above 
        // the field midpoint, otherwise the bottom pass.
        Translation2d targetPos = Util4828.getLockOnTargetPosition(robotPose);
        
        // Calculating parameters to shoot from anywhere.
        double distanceToTarget = robotPose.getTranslation().getDistance(targetPos);
        double targetVelocity = isScoring ? launchVelocityMap.get(distanceToTarget) : passVelocityMap.get(distanceToTarget);
        double targetHoodPosition = isScoring ? launchHoodPositionMap.get(distanceToTarget) : ShooterConstants.HOOD_MAX_POSITION;

        return new LaunchParameters(targetVelocity, targetHoodPosition);
    }

}
