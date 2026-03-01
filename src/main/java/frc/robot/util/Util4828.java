package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

import frc.robot.Constants;

public class Util4828 {

    /*** Game specific utility functions ***/
    public static Translation2d getHubLocation() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (alliance == Alliance.Red)
            ? Constants.FieldConstants.RED_HUB_CENTER
            : Constants.FieldConstants.BLUE_HUB_CENTER;
    }

    public static Translation2d getPassLocation(Pose2d robotPose) {
        boolean aim_top = robotPose.getY() > Constants.FieldConstants.FIELD_MIDPOINT_Y;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (alliance == Alliance.Red)
            return aim_top ? Constants.FieldConstants.RED_PASS_TOP : Constants.FieldConstants.RED_PASS_BOTTOM;
        else 
            return aim_top ? Constants.FieldConstants.BLUE_PASS_TOP : Constants.FieldConstants.BLUE_PASS_BOTTOM; 
    }

    public static boolean isInAllianceZone(Pose2d robotPose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red) {
            return robotPose.getX() > Constants.FieldConstants.RED_HUB_CENTER.getX();
        }
        else {
            return robotPose.getX() < Constants.FieldConstants.BLUE_HUB_CENTER.getX();
        }
    }

    public static boolean isInTopHalfOfField(Pose2d robotPose) {
        return robotPose.getY() > Constants.FieldConstants.FIELD_MIDPOINT_Y;
    }

    public static Translation2d getLockOnTargetPosition(Pose2d robotPose) {
        // If we're on our half of the field, lock to hub
        if (isInAllianceZone(robotPose)) {
            return getHubLocation();
        }
        
        // Otherwise, we're passing, lock to passing position
        return getPassLocation(robotPose);

    }

    /*** Generic utility functions ***/
    public static double metersPerSecondToWheelRPS(
            double metersPerSecond,
            double wheelDiameterMeters
    ) {
        double wheelCircumference = Math.PI * wheelDiameterMeters;
        return metersPerSecond / wheelCircumference;
    }

    public static Rotation2d averageRotation(Rotation2d[] rotations, double[] weights) {
        double sumCos = 0;
        double sumSin = 0;
        double totalWeight = 0;

        for (int i = 0; i < rotations.length; i++) {
            sumCos += Math.cos(rotations[i].getRadians()) * weights[i];
            sumSin += Math.sin(rotations[i].getRadians()) * weights[i];
            totalWeight += weights[i];
        }
        return new Rotation2d(sumSin / totalWeight, sumCos / totalWeight);
    }


    public static String formatPose(Pose2d pose) {
        return String.format(
            "x: %.3f  y: %.3f  rot: %.3f°",
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees()
        );
    }

    public static String formatMatrix3x1(Matrix<N3, N1> m) {
        return String.format(
            "[%.3f, %.3f, %.3f]",
            m.get(0, 0),
            m.get(1, 0),
            m.get(2, 0)
        );
    }

    public static Pose2d getAprilTagPose(int id) {
        Optional<edu.wpi.first.math.geometry.Pose3d> pose3dOpt = Constants.FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(id);
        if (pose3dOpt.isPresent()) {
            return pose3dOpt.get().toPose2d();
        } else {
            return null;
        }
    }

    public static void publishAprilTags(Field2d field) {
        // NetworkTable to hold the list of tags
        NetworkTable tagTable = NetworkTableInstance.getDefault().getTable("AprilTags");

        int index = 0; // List index
        for (AprilTag tag : Constants.FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTags()) {
            Pose3d tagPose3d = Constants.FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tag.ID).get();

            if (tagPose3d != null) {
                Pose2d tagPose2d = tagPose3d.toPose2d();

                // --- Field2D visualization ---
                String fieldName = String.format("%d - (%.2f, %.2f)", tag.ID,
                                                tagPose2d.getX(), tagPose2d.getY());
                FieldObject2d tagObject = field.getObject(fieldName);
                tagObject.setPose(tagPose2d);

                // --- Prepare string for ListLayout ---
                double xMeters = tagPose2d.getX();
                double yMeters = tagPose2d.getY();
                double xInches = xMeters * 39.3701;
                double yInches = yMeters * 39.3701;

                String tagInfo = String.format(
                    "Tag %d: (%.2f m, %.2f m) / (%.2f in, %.2f in)",
                    tag.ID, xMeters, yMeters, xInches, yInches
                );

                // Publish each tag as a separate entry in the table
                tagTable.getEntry(Integer.toString(index)).setString(tagInfo);
                index++;
            }
        }
    }

    /**
     * Calculates a Pose2d for the robot given an AprilTag ID, robot dimensions, and desired offset.
     *
     * @param tagId        The ID of the AprilTag
     * @param xOffsetMeters Distance from tag face along tag forward direction (positive = away from tag)
     * @param yOffsetMeters Lateral offset from tag center (positive = right of tag, negative = left)
     * @param frontToCenterMeters Distance from robot front to center
     * @param faceTag      If true, robot faces toward the tag; if false, faces away
     * @return Pose2d representing the robot’s starting position
     */
    public static Pose2d calculateRobotPoseFromTagId(
            int tagId,
            double xOffsetMeters,
            double yOffsetMeters,
            double frontToCenterMeters,
            boolean faceTag
    ) {
        // Verify tag exists
        if (!Constants.FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTags().stream().anyMatch(tag -> tag.ID == tagId)) {
            DriverStation.reportWarning("AprilTag ID " + tagId + " not found in layout", false);
            return null;
        }

        Pose3d tagPose3d = Constants.FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagId).orElse(null);
        if (tagPose3d == null) {
            DriverStation.reportWarning("Pose for tag " + tagId + " not found", false);
            return null;
        }

        // Total offset along tag forward
        double totalXOffset = xOffsetMeters + frontToCenterMeters;

        Pose2d tagPose2d = tagPose3d.toPose2d();

        // Determine direction for X offset
        Rotation2d forwardRotation = faceTag
                ? tagPose2d.getRotation()
                : tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180.0));

        // X offset along tag forward
        double offsetX = totalXOffset * forwardRotation.getCos();
        double offsetY = totalXOffset * forwardRotation.getSin();

        // Y offset perpendicular to tag forward (rotate 90 deg CCW for local left)
        double lateralX = -yOffsetMeters * forwardRotation.getSin();  // -sin for left
        double lateralY = yOffsetMeters * forwardRotation.getCos();   // cos for left

        double robotX = tagPose2d.getX() + offsetX + lateralX;
        double robotY = tagPose2d.getY() + offsetY + lateralY;

        // Robot rotation
        Rotation2d robotRotation = faceTag
                ? tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180.0))
                : tagPose2d.getRotation();

        return new Pose2d(robotX, robotY, robotRotation);
    }

}