package frc.robot.subsystems.shooter;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.util.Util4828;

public class LaunchCalculator {
    private static LaunchCalculator instance;

    // private final LinearFilter launchVelocityFilter = 
    //     LinearFilter.movingAverage((int) (ShooterConstants.SECONDS_OF_DATA_TO_AVERAGE / 0.02));
    // private final LinearFilter launchAngleFilter = 
    //     LinearFilter.movingAverage((int) (ShooterConstants.SECONDS_OF_DATA_TO_AVERAGE / 0.02));

    // private double calculatedVelocity;
    // private double lastHoodPosition;
    // private double calculatedHoodPosition;
    // private boolean isScoring;

    public static LaunchCalculator getInstance() {
        if (instance == null) {
            instance = new LaunchCalculator();
        }
        return instance;
    }

    public record LaunchParameters(double velocityMPS, double hoodPosition) {}

    private LaunchParameters latestParameters = null;

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

        latestParameters = new LaunchParameters(targetVelocity, targetHoodPosition);
        return latestParameters;
    }

}
