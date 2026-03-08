package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PoseSupplier;
import frc.robot.util.Util4828;
import frc.robot.util.PoseSupplier.Zone;


public class LaunchCalculator {

    /** PoseSupplier to get the robot pose */
    private final PoseSupplier poseSupplier;

    public LaunchCalculator(PoseSupplier poseSupplier) {
        this.poseSupplier = poseSupplier;

        SmartDashboard.putString("Shot Mode", "Anywhere");
    }

    public record LaunchParameters(double velocityMPS, double hoodPosition) {}

    // maps for scoring
    private final InterpolatingDoubleTreeMap launchVelocityMap = ShooterConstants.SHOOT_VELOCITY_MAP;
    private final InterpolatingDoubleTreeMap launchHoodPositionMap = ShooterConstants.SHOOT_HOOD_POSITION_MAP;

    // maps for passing
    private static final InterpolatingDoubleTreeMap passVelocityMap = ShooterConstants.PASS_VELOCITY_MAP;

    private Mode currentMode = Mode.SHOOT_FROM_ANYWHERE;
    private enum Mode {
        HUB_SHOT_ONLY,      //hardcoded for close range hub shots
        FAR_SHOT_ONLY,      //hardcoded for far range corner shots
        SHOOT_FROM_ANYWHERE //dynamically adjust based on distance
    }

    public void toggleHubShotMode() {
        if (currentMode == Mode.HUB_SHOT_ONLY)
            enterShootFromAnywhereMode();
        else
            enterHubShotMode();
    }
    public String getMode() {
        if (currentMode == Mode.HUB_SHOT_ONLY) {
            return "Hub Shot Only";
        }
        return "Shoot From Anywhere";
    }

    public void toggleFarShotMode() {
        if (currentMode == Mode.FAR_SHOT_ONLY)
            enterShootFromAnywhereMode();
        else
            enterFarShotMode();
    }

    public void enterHubShotMode() {
        currentMode = Mode.HUB_SHOT_ONLY;
        SmartDashboard.putString("Shot Mode", "Hub");
    }

    public void enterShootFromAnywhereMode() {
        currentMode = Mode.SHOOT_FROM_ANYWHERE;
        SmartDashboard.putString("Shot Mode", "Anywhere");
    }

    public void enterFarShotMode() {
        currentMode = Mode.FAR_SHOT_ONLY;
        SmartDashboard.putString("Shot Mode", "Far");
    }

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
        double distanceToTargetInches = Units.metersToInches(robotPose.getTranslation().getDistance(targetPos)) - 5.0;
        if (currentMode == Mode.HUB_SHOT_ONLY && isScoring) // if we're in hub shot mode, use distance to hub specifically
            distanceToTargetInches = 39.851142;
        if (currentMode == Mode.FAR_SHOT_ONLY && isScoring)
            distanceToTargetInches = 205.851142;
        double targetVelocity = isScoring ? launchVelocityMap.get(distanceToTargetInches) : passVelocityMap.get(distanceToTargetInches);
        double targetHoodPosition = isScoring ? launchHoodPositionMap.get(distanceToTargetInches) : ShooterConstants.HOOD_MAX_POSITION;

        // debugging - publish info
        SmartDashboard.putNumber("Tuning/Launch/DistanceToTarget", distanceToTargetInches);
        SmartDashboard.putNumber("Tuning/Launch/TargetVelocity", targetVelocity);
        SmartDashboard.putNumber("Tuning/Launch/TargetHood", targetHoodPosition);

        return new LaunchParameters(targetVelocity, targetHoodPosition);
    }
}
