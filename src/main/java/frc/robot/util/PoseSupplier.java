package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class PoseSupplier {
    private final CommandSwerveDrivetrain drivetrain;
    private Zone currentZone = Zone.SCORING_ZONE;

    /** Enum describing the zone the robot is in */
    public enum Zone {
        /** The robot is in our alliance zone where it can score */
        SCORING_ZONE,
        /** The robot is in the neutral zone */
        NEUTRAL_ZONE,
        /** The robot is in the opponent's alliance zone */
        OPPONENT_ZONE
    }

    public PoseSupplier(CommandSwerveDrivetrain drivetrain) {        
        this.drivetrain = drivetrain;
    }

    /** Returns a formatted string of the x, y, theta */
    public static String prettyPose(Pose2d pose) {
        return String.format("(%01.2f, %01.2f @ %01.1f)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    /** Returns the current heading of the robot */
    public Rotation2d getHeading() {
        return drivetrain.getState().Pose.getRotation();
    }

    /** Returns the current pose of the robot */
    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }
    
    /** Returns the current zone of the robot */
    public Zone getZone() {
        if (getPose().getX() < Constants.FieldConstants.BLUE_ALLIANCE_ZONE) {
            if (DriverStation.getAlliance().orElse(Alliance.Blue)  == Alliance.Blue)
                currentZone = Zone.SCORING_ZONE;
            else
                currentZone = Zone.OPPONENT_ZONE;
        }
        else if (getPose().getX() > Constants.FieldConstants.RED_ALLIANCE_ZONE) {
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
                currentZone = Zone.SCORING_ZONE;
            else
                currentZone = Zone.OPPONENT_ZONE;
        }
        else {
            currentZone = Zone.NEUTRAL_ZONE;
        }
        return currentZone;
    }

    /** Returns the distance of the robot to the given translation */
    public double distanceTo(Translation2d target) {
        return getPose().getTranslation().getDistance(target);
    }
    /** Returns the distance of the robot to the given position */
    public double distanceTo(Pose2d target) {
        return distanceTo(target.getTranslation());
    }
    /** Returns the distance of the robot to the center of the hub */
    public double distanceToHubCenter() {
        return distanceTo(Constants.FieldConstants.BLUE_HUB_CENTER);
    }
}
