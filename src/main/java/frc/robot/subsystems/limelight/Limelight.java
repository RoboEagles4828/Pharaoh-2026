package frc.robot.subsystems.limelight;

import java.util.Arrays;
import java.util.Comparator;

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

        // get the latest pose estimate (using MegaTag2) from the camera
        mostRecentPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        mostRecentPoseStandardDeviation = calculateStandardDeviationForEstimate(mostRecentPoseEstimate);
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
        SmartDashboard.putString(String.format("%s/%s"),
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

    // Tunable constants (start here, then adjust on a real field)
    private static Matrix<N3, N1> calculateStandardDeviationForEstimate(LimelightHelpers.PoseEstimate pose) {
        // Safety fallback — if pose is somehow invalid
        if (pose == null || pose.rawFiducials.length == 0) {
            return VecBuilder.fill(LimelightConstants.STD_MAX_XY, LimelightConstants.STD_MAX_XY, LimelightConstants.STD_MAX_THETA);
        }

        // Pick the best (lowest ambiguity) fiducial
        RawFiducial bestTag = Arrays.stream(pose.rawFiducials)
                .min((a, b) -> Double.compare(a.ambiguity, b.ambiguity))
                .orElse(null);

        // Error case, if we can't get a best pose somehow
        if (bestTag == null) {
            return VecBuilder.fill(LimelightConstants.STD_MAX_XY, LimelightConstants.STD_MAX_XY, LimelightConstants.STD_MAX_THETA);
        }

        double distance = bestTag.distToCamera;
        int tagCount = pose.tagCount;

        /*
         * Translation uncertainty
         * - Grows roughly with distance^2
         * - Multi-tag solves reduce uncertainty significantly
         */
        double xyStd = 0.1 + 0.02 * distance * distance;

        /*
         * Rotation uncertainty
         * - Worse than translation
         * - Grows linearly with distance
         */
        double thetaStd = Math.toRadians(5 + 2.0 * distance);

        // Improve confidence if we see multiple tags
        if (tagCount >= 2) {
            xyStd *= 0.5;
            thetaStd *= 0.5;
        }

        // Clamp to sane bounds
        xyStd = MathUtil.clamp(xyStd, LimelightConstants.STD_MIN_XY, LimelightConstants.STD_MAX_XY);
        thetaStd = MathUtil.clamp(thetaStd, LimelightConstants.STD_MIN_THETA, LimelightConstants.STD_MAX_THETA);

        return VecBuilder.fill(xyStd, xyStd, thetaStd);
    }
}
