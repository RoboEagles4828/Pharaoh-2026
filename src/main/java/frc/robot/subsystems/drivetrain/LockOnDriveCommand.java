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
import frc.robot.util.Util4828;

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
	/** Target position to lock on to */
	private final Translation2d targetPosition;

	/** Profiled PID Controller for rotation */
	private ProfiledPIDController headingPID;

	/**
	 * If the command should end (isFinished -> true) when we are within the aim tolerance.
	 * This is used for autonomous. During teleop, we want the command to run continuously so
	 * the driver can hold and drive while locked on, but in auto we just want to lock on and then
	 * move to the next part of the auto sequence.
	 */
	private final boolean shouldAutomaticallyEnd;

	/** CTRE swerve drive request with appropriate deadbands and control type for lock-on driving */
	private final SwerveRequest.FieldCentric driveRequest =
		new SwerveRequest.FieldCentric()
			.withDeadband(DrivetrainConstants.MAX_SPEED * DrivetrainConstants.DEADBAND)
			.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * DrivetrainConstants.ROTATIONAL_DEADBAND)
      		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


	public LockOnDriveCommand(
		CommandSwerveDrivetrain drivetrain,
		CommandXboxController controller,
		boolean shouldAutomaticallyEnd
	) {
		this.drivetrain = drivetrain;
		this.controller = controller;

		Pose2d robotPose = drivetrain.getState().Pose;
		this.targetPosition = Util4828.getLockOnTargetPosition(robotPose).minus(robotPose.getTranslation());

		this.shouldAutomaticallyEnd = shouldAutomaticallyEnd;

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

	/** Returns if the robot is within tolerance of the angle */
	// TODO might be unnecessary because you can just set the tolerance of the pid controller
	private boolean isWithinTolerance() {
		/** Current robot pose */
		Pose2d robotPose = drivetrain.getState().Pose;

		/** Vector from robot to hub */
		Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());

		/** Desired heading */
		Rotation2d desiredHeading = toTarget.getAngle();
		Rotation2d currentHeading = robotPose.getRotation();

		return Math.abs(desiredHeading.minus(currentHeading).getRadians()) < Math.toRadians(LockOnDriveConstraints.AIM_TOLERANCE_DEGREES);
	}

	@Override
	public void execute() {
		/** Current robot pose */
		Pose2d robotPose = drivetrain.getState().Pose;

		/** Vector from robot to hub */
		Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());

		/** Desired heading */
		Rotation2d desiredHeading = toTarget.getAngle();
		Rotation2d currentHeading = robotPose.getRotation();

		/** Calculated rotational velocity */
		double omega = headingPID.calculate(currentHeading.getRadians(), desiredHeading.getRadians());

		// Optional aim deadband (prevents jitter when lined up)
		if (isWithinTolerance()) {
			omega = 0.0;
		}

		// Apply CTRE request
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
		// Automatically terminating lockon whn we're in auton
		if (shouldAutomaticallyEnd && isWithinTolerance()) {
			return true;
		}

		// Runs while button is held so never finishes on its own during teleop
		return false;
	}
}
