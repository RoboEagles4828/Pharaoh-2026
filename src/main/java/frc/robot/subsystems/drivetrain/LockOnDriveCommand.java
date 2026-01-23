package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class LockOnDriveCommand extends Command {

	private final CommandSwerveDrivetrain drivetrain;

	// Driver translation input (field-relative)
	private final DoubleSupplier xSupplier;
	private final DoubleSupplier ySupplier;

	// Target location
	private final Translation2d hubPosition;

	// Rotation controller
	private final ProfiledPIDController headingPID;

	// CTRE drive request
	private final SwerveRequest.FieldCentric driveRequest =
		new SwerveRequest.FieldCentric()
			.withDeadband(0.05)
			.withRotationalDeadband(0.05);

	// ===== TUNING CONSTANTS =====
	private static final double kP = 4.0; // TODO tune
	private static final double kI = 0.0;
	private static final double kD = 0.25; // TODO tune

	private static final double MAX_OMEGA = 6.0; // rad/s (reasonable for swerve)
	private static final double MAX_ALPHA = 12.0; // rad/s² (example acceleration limit)
	private static final double AIM_TOLERANCE_RAD = Math.toRadians(1.5);

	public LockOnDriveCommand(
		CommandSwerveDrivetrain drivetrain,
		DoubleSupplier xSupplier,
		DoubleSupplier ySupplier,
		Translation2d hubPosition
	) {
		this.drivetrain = drivetrain;
		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.hubPosition = hubPosition;

		// Create a trapezoid-profiled PID controller
		this.headingPID = new ProfiledPIDController(
			kP,
			kI,
			kD,
			new TrapezoidProfile.Constraints(MAX_OMEGA, MAX_ALPHA)
		);

		// Enable wraparound for circular heading
		this.headingPID.enableContinuousInput(-Math.PI, Math.PI);

		addRequirements(drivetrain);
	}

    @Override
    public void initialize() {
        Pose2d robotPose = drivetrain.getState().Pose;
        headingPID.reset(robotPose.getRotation().getRadians());
    }


	@Override
	public void execute() {
		// === Current robot pose ===
		Pose2d robotPose = drivetrain.getState().Pose;

		// === Vector from robot to hub ===
		Translation2d toHub = hubPosition.minus(robotPose.getTranslation());

		// === Desired heading ===
		Rotation2d desiredHeading = toHub.getAngle();
		Rotation2d currentHeading = robotPose.getRotation();

		// === Rotation PID ===
		double omega = headingPID.calculate(
			currentHeading.getRadians(),
			desiredHeading.getRadians()
		);

		// Clamp rotational velocity
		omega = MathUtil.clamp(omega, -MAX_OMEGA, MAX_OMEGA);

		// Optional aim deadband (prevents jitter when lined up)
		if (Math.abs(
			desiredHeading.minus(currentHeading).getRadians()
		) < AIM_TOLERANCE_RAD) {
			omega = 0.0;
		}

		// === Driver translation ===
		double vx = xSupplier.getAsDouble();
		double vy = ySupplier.getAsDouble();

		// === Apply CTRE request ===
		drivetrain.setControl(
			driveRequest
				.withVelocityX(vx)
				.withVelocityY(vy)
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
		// Runs while button is held
		return false;
	}
}
