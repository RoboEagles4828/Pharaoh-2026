package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drivetrain.DrivetrainConstants.LockOnDriveConstraints;
import frc.robot.util.TunableNumber;

public class LockOnDriveCommand extends Command {
	// Tunable PID constants and constraints for lock-on behavior
	private static TunableNumber kP = new TunableNumber("Tuning/LockOn/PID_P", LockOnDriveConstraints.kP);
	private static TunableNumber kI = new TunableNumber("Tuning/LockOn/PID_I", LockOnDriveConstraints.kI);
	private static TunableNumber kD = new TunableNumber("Tuning/LockOn/PID_D", LockOnDriveConstraints.kD);
	// Tunable constraints for the profiled PID controller
	private static TunableNumber maxAngularVelocity = new TunableNumber("Tuning/LockOn/MaxRotVelo", LockOnDriveConstraints.MAX_ROTATIONAL_VELOCITY);
	private static TunableNumber maxAngularAcceleration = new TunableNumber("Tuning/LockOn/MaxRotAccel", LockOnDriveConstraints.MAX_ROTATIONAL_ACCELERATION);
	private static TunableNumber aimToleranceDeg = new TunableNumber("Tuning/LockOn/Tolerance", LockOnDriveConstraints.AIM_TOLERANCE_DEGREES);

	private final CommandSwerveDrivetrain drivetrain;
	/** Driver translation input (field-relative) */
	private final CommandXboxController controller;
	/** Target position to lock on to: the center of the hub */
	private final Translation2d hubPosition;
	/** Profiled PID Controller for rotation */
	private ProfiledPIDController headingPID;

	/** CTRE swerve drive request with appropriate deadbands and control type for lock-on driving */
	private final SwerveRequest.FieldCentric driveRequest =
		new SwerveRequest.FieldCentric()
			.withDeadband(DrivetrainConstants.MAX_SPEED * DrivetrainConstants.DEADBAND)
			.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * DrivetrainConstants.ROTATIONAL_DEADBAND)
      		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


	public LockOnDriveCommand(
		CommandSwerveDrivetrain drivetrain,
		CommandXboxController controller,
		Translation2d hubPosition
	) {
		this.drivetrain = drivetrain;
		this.controller = controller;
		this.hubPosition = hubPosition;

		addRequirements(drivetrain);
	}

    @Override
    public void initialize() {
		// Create a trapezoid-profiled PID controller
		this.headingPID = new ProfiledPIDController(
			kP.get(),
			kI.get(),
			kD.get(),
			new TrapezoidProfile.Constraints(
				maxAngularVelocity.get(),
				maxAngularAcceleration.get())
		);
		
		// Enable wraparound for circular heading
		this.headingPID.enableContinuousInput(-Math.PI, Math.PI);
		this.headingPID.setTolerance(Math.toRadians(aimToleranceDeg.get()));

        Pose2d robotPose = drivetrain.getState().Pose;
        headingPID.reset(robotPose.getRotation().getRadians());
    }


	@Override
	public void execute() {
		/** Current robot pose */
		Pose2d robotPose = drivetrain.getState().Pose;

		/** Vector from robot to hub */
		Translation2d toHub = hubPosition.minus(robotPose.getTranslation());

		/** Desired heading */
		Rotation2d desiredHeading = toHub.getAngle();
		Rotation2d currentHeading = robotPose.getRotation();

		/** Calculated rotational velocity */
		double omega = headingPID.calculate(currentHeading.getRadians(), desiredHeading.getRadians());

		// // Optional aim deadband (prevents jitter when lined up)
		// if (Math.abs(
		// 	desiredHeading.minus(currentHeading).getRadians()
		// ) < Math.toRadians(AIM_TOLERANCE_DEGREES)) {
		// 	omega = 0.0;
		// }

		drivetrain.setControl(
			driveRequest
				.withVelocityX(controller.getLeftY() * DrivetrainConstants.MAX_SPEED)
				.withVelocityY(controller.getLeftX() * DrivetrainConstants.MAX_SPEED)
				.withRotationalRate(omega)
		);
	}

	@Override
	public void end(boolean interrupted) {
		// Stop motion cleanly
		drivetrain.setControl(
			driveRequest
				.withVelocityX(0.0)
				.withVelocityY(0.0)
				.withRotationalRate(0.0)
		);
	}

	@Override
	public boolean isFinished() {
		// Runs while button is held so never finishes on its own
		return false;
	}
}
