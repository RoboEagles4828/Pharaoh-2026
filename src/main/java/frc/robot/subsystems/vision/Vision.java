package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.util.PoseSupplier;

public class Vision extends SubsystemBase {

    private final Limelight limelightForward;
    private final Limelight limelightSide;
    private final Limelight limelightBack;

    private CommandSwerveDrivetrain drivetrain;

    public Vision(CommandSwerveDrivetrain drivetrain, PoseSupplier poseSupplier) {
        limelightForward = new Limelight(VisionConstants.LIMELIGHT_FORWARD_NAME, poseSupplier);
        limelightSide = new Limelight(VisionConstants.LIMELIGHT_SIDE_NAME, poseSupplier);
        limelightBack = new Limelight(VisionConstants.LIMELIGHT_BACK_NAME, poseSupplier);

        this.drivetrain = drivetrain;

        SmartDashboard.putBoolean(VisionConstants.NT_USE_VISION_TOGGLE, true);

        // visually draw the two hub centers on field object
        Constants.FieldConstants.FIELD.getObject("Red Hub").setPose(new Pose2d(Constants.FieldConstants.RED_HUB_CENTER, new Rotation2d(0)));
        Constants.FieldConstants.FIELD.getObject("Blue Hub").setPose(new Pose2d(Constants.FieldConstants.BLUE_HUB_CENTER, new Rotation2d(0)));
    }

    /** Updates a limelight's estimate and feed it to the drivetrain if it's valid */
    private void processLimelight(Limelight limelight) {
        limelight.updateEstimate(); 

        if (SmartDashboard.getBoolean(VisionConstants.NT_USE_VISION_TOGGLE, true)) {
            if (limelight.isPoseEstimateGood()) {
                PoseEstimate estimate = limelight.getPoseEstimate();
                drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, limelight.getPoseStandardDeviation().times(VisionConstants.VISION_TRUST_SCALAR));
                // drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, limelight.getPoseStandardDeviation().times(VisionConstants.VISION_TRUST_SCALAR));
            }
        }
    }

    @Override
    public void periodic() {
        processLimelight(limelightForward);
        processLimelight(limelightSide);
        processLimelight(limelightBack);
    }

}