package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class PoseSubsystem {
    private static PoseSubsystem instance;
    private final CommandSwerveDrivetrain drivetrain;

    public enum Zone {
        SCORING_ZONE,
        NEUTRAL_ZONE,
        OPPONENT_ZONE
    }

    public PoseSubsystem(CommandSwerveDrivetrain drivetrain) {
        if (instance == null) {
            instance = new PoseSubsystem(drivetrain);
        }
        
        this.drivetrain = drivetrain;

    }

    public static PoseSubsystem getInstance() {
        return instance;
    }

    public static String prettyPose(Pose2d pose) {
        return String.format("(%01.2f, %01.2f @ %01.1f)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public Rotation2d getHeading() {
        return drivetrain.getState().Pose.getRotation();
    }
    public void setHeading(Rotation2d newHeading) {
        drivetrain.seedFieldCentric(newHeading);
    }
    public void resetHeading() {
        setHeading(Rotation2d.fromDegrees(0));
    }

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }
    public void setPose(Pose2d newPose) {
        drivetrain.resetPose(newPose);
    }

    public static double distanceTo(Translation2d target) {
        return getInstance().getPose().getTranslation().getDistance(target);
    }
}
