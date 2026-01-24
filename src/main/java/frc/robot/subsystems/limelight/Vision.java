package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
    private static final String NT_USE_VISION_BUTTON = "LL Use Vision";

    private final Limelight limelightOne;
    private final Limelight limelightTwo;

    private final CommandSwerveDrivetrain drivetrain;

    public Vision(CommandSwerveDrivetrain drive) {
        drivetrain = drive;

        limelightOne = new Limelight(LimelightConstants.LIMELIGHT_ONE_NAME, drivetrain);
        limelightTwo = new Limelight(LimelightConstants.LIMELIGHT_TWO_NAME, drivetrain);

        SmartDashboard.putBoolean(NT_USE_VISION_BUTTON, true);

        // visually draw the two hub centers on field object
        Constants.FieldConstants.FIELD.getObject("Red Hub").setPose(new Pose2d(Constants.FieldConstants.RED_HUB_CENTER, new Rotation2d(0)));
        Constants.FieldConstants.FIELD.getObject("Blue Hub").setPose(new Pose2d(Constants.FieldConstants.BLUE_HUB_CENTER, new Rotation2d(0)));
    }

    @Override
    public void periodic() {
        limelightOne.update();
        limelightTwo.update();
        
        // Get pose estimates from the limelights
        PoseEstimate limelightOneEstimate = null;
        PoseEstimate limelightTwoEstimate = null;

        if (limelightOne.isPoseEstimateGood()) {
            limelightOneEstimate = limelightOne.getPoseEstimate();
        }

        if (limelightTwo.isPoseEstimateGood()) {
            limelightTwoEstimate = limelightTwo.getPoseEstimate();
        }

        // TODO - Filter the limelights or some sort of fusion
        PoseEstimate fused = limelightOneEstimate;
        

        // If vision is enabled on dashboard, feed this reading to the drivetrain
        if (fused != null && SmartDashboard.getBoolean(NT_USE_VISION_BUTTON, true)) {
            drivetrain.addVisionMeasurement(fused.pose, fused.timestampSeconds);
        }

    }

}