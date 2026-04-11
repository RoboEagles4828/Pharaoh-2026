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
        /** Hardcoded for close range hub shots */
        HUB_SHOT_ONLY,
        /** Hardcoded for mid-range tower shots */
        TOWER_SHOT_ONLY,
        /** Hardcoded for far range corner shots */
        FAR_SHOT_ONLY,
        /** Dynamically adjust launch calculations based on distance */
        SHOOT_FROM_ANYWHERE
    }

    public boolean doesModeLockOn() {
        if (currentMode == Mode.HUB_SHOT_ONLY || currentMode == Mode.TOWER_SHOT_ONLY)
            return false;
        return true;
    }

    /** Returns the current mode of the launch calculator */
    public String getMode() {
        if (currentMode == Mode.HUB_SHOT_ONLY) {
            return "Hub Shot Only";
        }
        else if (currentMode == Mode.FAR_SHOT_ONLY) {
            return "Far Shot Only";
        }
        return "Shoot From Anywhere";
    }

    public void toggleHubShotMode() {
        if (currentMode == Mode.HUB_SHOT_ONLY)
            enterShootFromAnywhereMode();
        else
            enterHubShotMode();
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
    public void enterTowerShotMode() {
        currentMode = Mode.TOWER_SHOT_ONLY;
        SmartDashboard.putString("Shot Mode", "Tower");
    }
    public void enterFarShotMode() {
        currentMode = Mode.FAR_SHOT_ONLY;
        SmartDashboard.putString("Shot Mode", "Far");
    }
    public void enterShootFromAnywhereMode() {
        currentMode = Mode.SHOOT_FROM_ANYWHERE;
        SmartDashboard.putString("Shot Mode", "Anywhere");
    }
    

    /** Returns the shooter hood position and launch velocity based on the distance from the target position */
    public LaunchParameters getParameters() {
        Pose2d robotPose = poseSupplier.getPose();

        // Are we scoring or passing? If we're in our alliance zone, we're trying to score.
        boolean isScoring = poseSupplier.getZone() == Zone.SCORING_ZONE;

        Translation2d targetPos = Util4828.getLockOnTargetPosition(robotPose);
        
        // Calculating parameters to shoot from anywhere.
        double distanceToTargetInches = Units.metersToInches(robotPose.getTranslation().getDistance(targetPos)) - 2.0;
        double targetVelocity = isScoring ? launchVelocityMap.get(distanceToTargetInches) : passVelocityMap.get(distanceToTargetInches);
        double targetHoodPosition = isScoring ? launchHoodPositionMap.get(distanceToTargetInches) : ShooterConstants.HOOD_MAX_POSITION;

        if (currentMode == Mode.HUB_SHOT_ONLY) { // if we're in hub shot mode, use distance to hub specifically
            targetVelocity = ShooterConstants.HUB_SHOT_VELOCITY;
            targetHoodPosition = ShooterConstants.HUB_SHOT_HOOD;
        }
        else if (currentMode == Mode.TOWER_SHOT_ONLY){
            targetVelocity = ShooterConstants.TOWER_SHOT_VELOCITY;
            targetHoodPosition = ShooterConstants.TOWER_SHOT_HOOD;
        }
        else if (currentMode == Mode.FAR_SHOT_ONLY){
            targetVelocity = ShooterConstants.FAR_SHOT_VELOCITY;
            targetHoodPosition = ShooterConstants.FAR_SHOT_HOOD;
        }

        // debugging - publish info
        SmartDashboard.putNumber("Tuning/Launch/DistanceToTarget", distanceToTargetInches);
        SmartDashboard.putNumber("Tuning/Launch/TargetVelocity", targetVelocity);
        SmartDashboard.putNumber("Tuning/Launch/TargetHood", targetHoodPosition);

        return new LaunchParameters(targetVelocity, targetHoodPosition);
    }
}
