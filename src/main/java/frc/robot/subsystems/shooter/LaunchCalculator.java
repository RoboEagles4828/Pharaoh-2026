package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PoseSupplier;
import frc.robot.util.Util4828;
import frc.robot.util.PoseSupplier.Zone;


public class LaunchCalculator {

    /** PoseSupplier to get the robot pose */
    private final PoseSupplier poseSupplier;

    public LaunchCalculator(PoseSupplier poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public record LaunchParameters(double velocityMPS, double hoodPosition) {}

    // maps for scoring
    private final InterpolatingDoubleTreeMap launchVelocityMap = ShooterConstants.SHOOT_VELOCITY_MAP;
    private final InterpolatingDoubleTreeMap launchHoodPositionMap = ShooterConstants.SHOOT_HOOD_POSITION_MAP;

    // maps for passing
    private static final InterpolatingDoubleTreeMap passVelocityMap = ShooterConstants.PASS_VELOCITY_MAP;

    public LaunchParameters getParameters() {
        Pose2d robotPose = poseSupplier.getPose();

        // Are we scoring or passing? If we're in our alliance zone, we're trying to score.
        boolean isScoring = poseSupplier.getZone() == Zone.SCORING_ZONE;

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

        // debugging - publish info
        SmartDashboard.putNumber("Tuning/Launch/DistanceToTarget", distanceToTarget);
        SmartDashboard.putNumber("Tuning/Launch/TargetVelocity", targetVelocity);
        SmartDashboard.putNumber("Tuning/Launch/TargetHood", targetHoodPosition);

        return new LaunchParameters(targetVelocity, targetHoodPosition);
    }
}
