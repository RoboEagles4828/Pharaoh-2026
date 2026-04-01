package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PoseSupplier;
import frc.robot.util.Util4828;
import frc.robot.util.PoseSupplier.Zone;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;


public class LaunchCalculator {

    /** PoseSupplier to get the robot pose */
    private final PoseSupplier poseSupplier;
    private final CommandSwerveDrivetrain drivetrain;

    public LaunchCalculator(PoseSupplier poseSupplier, CommandSwerveDrivetrain drivetrain) {
        this.poseSupplier = poseSupplier;
        this.drivetrain = drivetrain;

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
        /** Hardcoded for far range corner shots */
        FAR_SHOT_ONLY,
        /** Dynamically adjust launch calculations based on distance */
        SHOOT_FROM_ANYWHERE
    }

    public boolean doesModeLockOn() {
        if (currentMode == Mode.HUB_SHOT_ONLY)
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
    public void enterShootFromAnywhereMode() {
        currentMode = Mode.SHOOT_FROM_ANYWHERE;
        SmartDashboard.putString("Shot Mode", "Anywhere");
    }
    public void enterFarShotMode() {
        currentMode = Mode.FAR_SHOT_ONLY;
        SmartDashboard.putString("Shot Mode", "Far");
    }

    /** Returns the shooter hood position and launch velocity based on the distance from the target position */
    public LaunchParameters getParameters() {
        Pose2d robotPose = poseSupplier.getPose();

        // Are we scoring or passing? If we're in our alliance zone, we're trying to score.
        boolean isScoring = poseSupplier.getZone() == Zone.SCORING_ZONE;

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, robotPose.getRotation());
        Translation2d targetPos = Util4828.getMovingLockOnPosition(robotPose, fieldSpeeds);
        
        // Calculating parameters to shoot from anywhere.
        double distanceToTargetInches = Units.metersToInches(robotPose.getTranslation().getDistance(targetPos)) - 2.0;
        double targetVelocity = isScoring ? launchVelocityMap.get(distanceToTargetInches) : passVelocityMap.get(distanceToTargetInches);
        double targetHoodPosition = isScoring ? launchHoodPositionMap.get(distanceToTargetInches) : ShooterConstants.HOOD_MAX_POSITION;

        if (currentMode == Mode.HUB_SHOT_ONLY) { // if we're in hub shot mode, use distance to hub specifically
            targetVelocity = ShooterConstants.HUB_SHOT_VELOCITY;
            targetHoodPosition = ShooterConstants.HUB_SHOT_HOOD;
            
        }
        else if (currentMode == Mode.FAR_SHOT_ONLY){
            targetVelocity = 20.57;
            targetHoodPosition = -1.181;
        }

        // debugging - publish info
        SmartDashboard.putNumber("Tuning/Launch/DistanceToTarget", distanceToTargetInches);
        SmartDashboard.putNumber("Tuning/Launch/TargetVelocity", targetVelocity);
        SmartDashboard.putNumber("Tuning/Launch/TargetHood", targetHoodPosition);

        return new LaunchParameters(targetVelocity, targetHoodPosition);
    }
}
