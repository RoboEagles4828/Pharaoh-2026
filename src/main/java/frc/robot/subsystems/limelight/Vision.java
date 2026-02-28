package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
    private static final String NT_USE_VISION_TOGGLE = "LL Use Vision";

    private final Limelight limelightOne;
    //private final Limelight limelightTwo;

    private final CommandSwerveDrivetrain drivetrain;

    public Vision(CommandSwerveDrivetrain drive) {
        drivetrain = drive;

        limelightOne = new Limelight(LimelightConstants.LIMELIGHT_ONE_NAME, drivetrain);
        //limelightTwo = new Limelight(LimelightConstants.LIMELIGHT_TWO_NAME, drivetrain);

        SmartDashboard.putBoolean(NT_USE_VISION_TOGGLE, true);

        // visually draw the two hub centers on field object
        Constants.FieldConstants.FIELD.getObject("Red Hub").setPose(new Pose2d(Constants.FieldConstants.RED_HUB_CENTER, new Rotation2d(0)));
        Constants.FieldConstants.FIELD.getObject("Blue Hub").setPose(new Pose2d(Constants.FieldConstants.BLUE_HUB_CENTER, new Rotation2d(0)));
    }

    @Override
    public void periodic() {
        processLimelight(limelightOne);
        //processLimelight(limelightTwo);
    }

    // Updates a limelight's estimate and feed it to the drivetrain if it's valid
    private void processLimelight(Limelight limelight) {
        limelight.updateEstimate(); 

        if (SmartDashboard.getBoolean(NT_USE_VISION_TOGGLE, true))
        {
            if (limelight.isPoseEstimateGood()) {
                PoseEstimate estimate = limelight.getPoseEstimate();
                drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, limelight.getPoseStandardDeviation());
            }
        }
    }

}