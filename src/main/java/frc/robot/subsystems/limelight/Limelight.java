package frc.robot.subsystems.limelight;

import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.limelight.LimelightHelpers.RawFiducial;
import frc.robot.util.Util4828;

public class Limelight extends SubsystemBase {
    private static final String NT_TX = "LL TX";
    private static final String NT_TY = "LL TY";
    private static final String NT_TAG_ID = "LL Tag ID";
    private static final String NT_POSE_MT1 = "LL Pose (MT1)";
    private static final String NT_POSE_MT2 = "LL Pose (MT2)";
    private static final String NT_IS_ESTIMATE_GOOD = "LL Is Estimate Good";
    private static final String NT_TIMESTAMP = "LL Timestamp";
    private static final String NT_USE_VISION_BUTTON = "LL Use Vision";

    private CommandSwerveDrivetrain drivetrain;

    public Limelight(CommandSwerveDrivetrain drive) {
        drivetrain = drive;

        SmartDashboard.putBoolean(NT_USE_VISION_BUTTON, true);
    }

    @Override
    public void periodic() {
        // feed the robot's current rotation to the limelight (required for MegaTag2 algorithm)
        if (drivetrain != null) {
            LimelightHelpers.SetRobotOrientation(LimelightConstants.LIMELIGHT_NAME, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        }

        // get the latest pose estimate (using MegaTag2) from the camera
        LimelightHelpers.PoseEstimate poseEstimateMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LIMELIGHT_NAME);
        LimelightHelpers.PoseEstimate poseEstimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LIMELIGHT_NAME);
        
        if (poseEstimateMT1 != null) {
            //Constants.FieldConstants.FIELD.getObject("MegaTag1").setPose(poseEstimateMT1.pose);
        }

        boolean isPoseEstimateGood = verifyPoseEstimate(poseEstimateMT2);
        if (isPoseEstimateGood) {
            // If vision is enabled on dashboard, feed this reading to the drivetrain
            if (SmartDashboard.getBoolean(NT_USE_VISION_BUTTON, true)) {
                drivetrain.addVisionMeasurement(poseEstimateMT2.pose, poseEstimateMT2.timestampSeconds);
            }
        }

        // Display the estimate on the field, or at (0, 0) if we have no estimate
        Constants.FieldConstants.FIELD.getObject("MegaTag2").setPose(
            isPoseEstimateGood ?
            poseEstimateMT2.pose :
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0))
        );

        Constants.FieldConstants.FIELD.getObject("Red Hub").setPose(new Pose2d(Constants.FieldConstants.RED_HUB_CENTER, new Rotation2d(0)));
        Constants.FieldConstants.FIELD.getObject("Blue Hub").setPose(new Pose2d(Constants.FieldConstants.BLUE_HUB_CENTER, new Rotation2d(0)));
        
        if (poseEstimateMT2 != null) {
            // pick the fiducial with lowest ambiguity
            RawFiducial bestTag = Arrays.stream(poseEstimateMT2.rawFiducials)
                .min(Comparator.comparingDouble(f -> f.ambiguity))
                .orElse(null);
            if (bestTag != null) {
                SmartDashboard.putNumber("Distance to Tag", bestTag.distToCamera);
                SmartDashboard.putNumber("Best Tag ID", bestTag.id);
            }
        }

        

        SmartDashboard.putNumber(NT_TX, LimelightHelpers.getTX(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putNumber(NT_TY, LimelightHelpers.getTY(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putNumber(NT_TAG_ID, LimelightHelpers.getFiducialID(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putString(NT_POSE_MT1, poseEstimateMT1 == null ? "NULL" : Util4828.formatPose(poseEstimateMT1.pose));
        SmartDashboard.putString(NT_POSE_MT2, poseEstimateMT2 == null ? "NULL" : Util4828.formatPose(poseEstimateMT2.pose));
        SmartDashboard.putBoolean(NT_IS_ESTIMATE_GOOD, isPoseEstimateGood);
        SmartDashboard.putNumber(NT_TIMESTAMP, poseEstimateMT2 == null ? -1 : poseEstimateMT2.timestampSeconds);
    }

    // Checks if a pose estimate (limelight reading) is of sufficient quality to be used.
    final static double AMBIGUITY_THRESHOLD = 0.7; //< Reject the pose if ambiguity is above this.
    final static double DISTANCE_THRESHOLD = 15.0; //< Reject the pose if distance is above this (meters). 
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
        if (bestTag.ambiguity > AMBIGUITY_THRESHOLD) {
            return false;
        }

        // If we're too far from the tag, reject
        if (bestTag.distToCamera > DISTANCE_THRESHOLD) {
            return false;
        }

        return true;
    }
}
