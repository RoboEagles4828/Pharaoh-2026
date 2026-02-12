package frc.robot.subsystems.limelight;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.limelight.LimelightHelpers.RawFiducial;
import frc.robot.util.Util4828;

public class Limelight {
    private static final String NT_TX = "TX";
    private static final String NT_TY = "TY";
    private static final String NT_TAG_ID = "Tag ID";
    private static final String NT_POSE_MT2 = "Pose (MT2)";
    private static final String NT_IS_ESTIMATE_GOOD = "Is Estimate Good";
    private static final String NT_STANDARD_DEVIATION = "Standard Dev";
    private static final String NT_TIMESTAMP = "Timestamp";

    private final String name;
    private final CommandSwerveDrivetrain drivetrain;

    private PoseEstimate mostRecentPoseEstimate = null;
    private Matrix<N3, N1> mostRecentPoseStandardDeviation = null;
    private boolean isMostRecentPoseEstimateGood = false;

    public Limelight(String limelightName, CommandSwerveDrivetrain drive) {
        name = limelightName;
        drivetrain = drive;
    }

    public void updateEstimate() {
        // feed the robot's current rotation to the limelight (required for MegaTag2
        // algorithm)
        if (drivetrain != null) {
            LimelightHelpers.SetRobotOrientation(name, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0,
                    0, 0);
        }

        // Get pose estimate from MegaTag2
        mostRecentPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        // Compute standard deviation based on spread of tags
        mostRecentPoseStandardDeviation = calculateStandardDeviationForEstimate(mostRecentPoseEstimate);

        // Check if best-fit estimate is good
        isMostRecentPoseEstimateGood = verifyPoseEstimate(mostRecentPoseEstimate);

        // Display the estimate on the field, or at (0, 0) if we have no estimate
        Constants.FieldConstants.FIELD.getObject(String.format("%s Pose Estimate", name)).setPose(
                isMostRecentPoseEstimateGood ? mostRecentPoseEstimate.pose
                        : new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

        SmartDashboard.putNumber(String.format("%s/%s", name, NT_TX), LimelightHelpers.getTX(name));
        SmartDashboard.putNumber(String.format("%s/%s", name, NT_TY), LimelightHelpers.getTY(name));
        SmartDashboard.putNumber(String.format("%s/%s", name, NT_TAG_ID), LimelightHelpers.getFiducialID(name));
        SmartDashboard.putString(String.format("%s/%s", name, NT_POSE_MT2),
                mostRecentPoseEstimate == null ? "NULL" : Util4828.formatPose(mostRecentPoseEstimate.pose));
        SmartDashboard.putBoolean(String.format("%s/%s", name, NT_IS_ESTIMATE_GOOD), isMostRecentPoseEstimateGood);
        SmartDashboard.putString(String.format("%s/%s", name, NT_STANDARD_DEVIATION),
                mostRecentPoseEstimate == null ? "NULL" : Util4828.formatMatrix3x1(mostRecentPoseStandardDeviation));
        SmartDashboard.putNumber(String.format("%s/%s", name, NT_TIMESTAMP),
                mostRecentPoseEstimate == null ? -1 : mostRecentPoseEstimate.timestampSeconds);
    }

    public PoseEstimate getPoseEstimate() {
        return mostRecentPoseEstimate;
    }

    public Matrix<N3, N1> getPoseStandardDeviation() {
        return mostRecentPoseStandardDeviation;
    }

    public boolean isPoseEstimateGood() {
        return isMostRecentPoseEstimateGood;
    }

    // Checks if a pose estimate (limelight reading) is of sufficient quality to be
    // used.
    private static boolean verifyPoseEstimate(PoseEstimate pose) {
        // if we have no estimate or see no tags, reject
        if (pose == null || pose.tagCount == 0 || pose.rawFiducials.length == 0) {
            return false;
        }

        // pick the fiducial with lowest ambiguity
        RawFiducial bestTag = Arrays.stream(pose.rawFiducials)
                .min(Comparator.comparingDouble(f -> f.ambiguity))
                .orElse(null);

        // If ambiguity is too high, reject
        if (bestTag.ambiguity > LimelightConstants.POSE_AMBIGUITY_THRESHOLD) {
            return false;
        }

        // If we're too far from the tag, reject
        if (bestTag.distToCamera > LimelightConstants.POSE_DISTANCE_THRESHOLD) {
            return false;
        }

        return true;
    }

    private static Matrix<N3, N1> calculateStandardDeviationForEstimate(LimelightHelpers.PoseEstimate pose) {
        if (pose == null || pose.rawFiducials.length == 0) {
            return VecBuilder.fill(LimelightConstants.STD_MAX_XY,
                    LimelightConstants.STD_MAX_XY,
                    LimelightConstants.STD_MAX_THETA);
        }

        Pose2d fusedPose = pose.pose;

        double xVar = 0.0;
        double yVar = 0.0;
        double thetaVar = 0.0;
        double weightSum = 0.0;

        for (RawFiducial tag : pose.rawFiducials) {
            Pose2d tagPose = Util4828.getAprilTagPose(tag.id);
            if (tagPose == null)
                continue; // Skip missing tags

            double weight = 1.0 / (tag.ambiguity + 0.01);

            double dx = tagPose.getX() - fusedPose.getX();
            double dy = tagPose.getY() - fusedPose.getY();
            double dtheta = tagPose.getRotation().minus(fusedPose.getRotation()).getRadians();

            xVar += weight * dx * dx;
            yVar += weight * dy * dy;
            thetaVar += weight * dtheta * dtheta;
            weightSum += weight;
        }

        if (weightSum == 0.0) {
            // No valid tags
            return VecBuilder.fill(LimelightConstants.STD_MAX_XY,
                    LimelightConstants.STD_MAX_XY,
                    LimelightConstants.STD_MAX_THETA);
        }

        double xStd = Math.sqrt(xVar / weightSum);
        double yStd = Math.sqrt(yVar / weightSum);
        double thetaStd = Math.sqrt(thetaVar / weightSum);

        xStd = MathUtil.clamp(xStd, LimelightConstants.STD_MIN_XY, LimelightConstants.STD_MAX_XY);
        yStd = MathUtil.clamp(yStd, LimelightConstants.STD_MIN_XY, LimelightConstants.STD_MAX_XY);
        thetaStd = MathUtil.clamp(thetaStd, LimelightConstants.STD_MIN_THETA, LimelightConstants.STD_MAX_THETA);

        return VecBuilder.fill(xStd, yStd, thetaStd);
    }

}
