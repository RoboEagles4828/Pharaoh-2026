package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.Util4828;

public class Limelight extends SubsystemBase {
    private static final String NT_TX = "LL TX";
    private static final String NT_TY = "LL TY";
    private static final String NT_TAG_ID = "LL Tag ID";
    private static final String NT_POSE_MT1 = "LL Pose (MT1)";
    private static final String NT_POSE_MT2 = "LL Pose (MT2)";
    private static final String NT_TIMESTAMP = "LL Timestamp";

    private CommandSwerveDrivetrain drivetrain;

    public Limelight(CommandSwerveDrivetrain drive) {
        drivetrain = drive;
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
            Constants.FieldConstants.FIELD.getObject("MegaTag1").setPose(poseEstimateMT1.pose);
        }

        if (poseEstimateMT2 != null) {
            Constants.FieldConstants.FIELD.getObject("MegaTag2").setPose(poseEstimateMT2.pose);
        }

        SmartDashboard.putNumber(NT_TX, LimelightHelpers.getTX(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putNumber(NT_TY, LimelightHelpers.getTY(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putNumber(NT_TAG_ID, LimelightHelpers.getFiducialID(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putString(NT_POSE_MT1, poseEstimateMT1 == null ? "NULL" : Util4828.formatPose(poseEstimateMT1.pose));
        SmartDashboard.putString(NT_POSE_MT2, poseEstimateMT2 == null ? "NULL" : Util4828.formatPose(poseEstimateMT2.pose));
        SmartDashboard.putNumber(NT_TIMESTAMP, poseEstimateMT2 == null ? -1 : poseEstimateMT2.timestampSeconds);
    }
}
