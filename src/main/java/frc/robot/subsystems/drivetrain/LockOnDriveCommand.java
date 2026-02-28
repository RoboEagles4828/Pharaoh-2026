package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.Util4828;

public class LockOnDriveCommand extends Command {
	private static final String NT_LOCK_ON_PID_P = "Tuning/LockOn/PID_P";
	private static final String NT_LOCK_ON_PID_I = "Tuning/LockOn/PID_I";
	private static final String NT_LOCK_ON_PID_D = "Tuning/LockOn/PID_D";
	private static final String NT_LOCK_ON_MAX_VELOCITY = "Tuning/LockOn/MaxRotVelo";
	private static final String NT_LOCK_ON_MAX_ACCELERATION = "Tuning/LockOn/MaxRotAccel";
	private static final String NT_LOCK_ON_TOLERANCE = "Tuning/LockOn/Tolerance";

	private final CommandSwerveDrivetrain drivetrain;

	// Driver translation input (field-relative)
	private final CommandXboxController controller;

	// Target location
	private final Translation2d targetPosition;

	// Rotation controller
	private ProfiledPIDController headingPID;

	// If the command should end (isFinished -> true) when we are within the aim tolerance.
	// This is used for autonomous. During teleop, we want the command to run continously
	// so that the driver can hold and drive while locked on.
	// But in auto, we just want to lock on and then move to the next part of the auto sequence.
	private final boolean shouldAutomaticallyEnd; 

	// CTRE drive request
	private final SwerveRequest.FieldCentric driveRequest =
		new SwerveRequest.FieldCentric()
			.withDeadband(DrivetrainConstants.MAX_SPEED * DrivetrainConstants.DEADBAND)
			.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * DrivetrainConstants.ROTATIONAL_DEADBAND)
      		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	// ===== TUNING CONSTANTS =====
	private static final double kP = 12.0; 
	private static final double kI = 0.0;
	private static final double kD = 0.5;
	private static final double MAX_ROTATIONAL_VELOCITY = 12.0; // rad/s
	private static final double MAX_ROTATIONAL_ACCELERATION = 18.0; // rad/s²
	private static final double AIM_TOLERANCE_DEGREES = 1.5; // degrees

	public LockOnDriveCommand(
		CommandSwerveDrivetrain drivetrain,
		CommandXboxController controller,
		boolean shouldAutomaticallyEnd
	) {
		this.drivetrain = drivetrain;
		this.controller = controller;

		Pose2d robotPose = drivetrain.getState().Pose;
		this.targetPosition = Util4828.getLockOnTargetPosition(robotPose).minus(robotPose.getTranslation());

		SmartDashboard.putNumber(NT_LOCK_ON_PID_P, kP);
		SmartDashboard.putNumber(NT_LOCK_ON_PID_I, kI);
		SmartDashboard.putNumber(NT_LOCK_ON_PID_D, kD);
		SmartDashboard.putNumber(NT_LOCK_ON_MAX_VELOCITY, MAX_ROTATIONAL_VELOCITY);
		SmartDashboard.putNumber(NT_LOCK_ON_MAX_ACCELERATION, MAX_ROTATIONAL_ACCELERATION);
		SmartDashboard.putNumber(NT_LOCK_ON_TOLERANCE, AIM_TOLERANCE_DEGREES);

		this.shouldAutomaticallyEnd = shouldAutomaticallyEnd;

		addRequirements(drivetrain);
	}

    @Override
    public void initialize() {
		// Create a trapezoid-profiled PID controller
		this.headingPID = new ProfiledPIDController(
			SmartDashboard.getNumber(NT_LOCK_ON_PID_P, kP),
			SmartDashboard.getNumber(NT_LOCK_ON_PID_I, kI),
			SmartDashboard.getNumber(NT_LOCK_ON_PID_D, kD),
			new TrapezoidProfile.Constraints(
				SmartDashboard.getNumber(NT_LOCK_ON_MAX_VELOCITY, MAX_ROTATIONAL_VELOCITY),
				SmartDashboard.getNumber(NT_LOCK_ON_MAX_ACCELERATION, MAX_ROTATIONAL_ACCELERATION))
		);
		
		// Enable wraparound for circular heading
		this.headingPID.enableContinuousInput(-Math.PI, Math.PI);

        Pose2d robotPose = drivetrain.getState().Pose;
        headingPID.reset(robotPose.getRotation().getRadians());
    }

	// Returns if the robot's
	private boolean isWithinTolerance() {
		// === Current robot pose ===
		Pose2d robotPose = drivetrain.getState().Pose;

		// === Vector from robot to hub ===
		Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());

		// === Desired heading ===
		Rotation2d desiredHeading = toTarget.getAngle();
		Rotation2d currentHeading = robotPose.getRotation();

		return Math.abs(desiredHeading.minus(currentHeading).getRadians()) < Math.toRadians(AIM_TOLERANCE_DEGREES);
	}

	@Override
	public void execute() {
		// === Current robot pose ===
		Pose2d robotPose = drivetrain.getState().Pose;

		// === Vector from robot to hub ===
		Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());

		// === Desired heading ===
		Rotation2d desiredHeading = toTarget.getAngle();
		Rotation2d currentHeading = robotPose.getRotation();

		// === Rotation PID ===
		double omega = headingPID.calculate(
			currentHeading.getRadians(),
			desiredHeading.getRadians()
		);

		// Optional aim deadband (prevents jitter when lined up)
		if (isWithinTolerance()) {
			omega = 0.0;
		}

		// === Apply CTRE request ===
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
		// automatically terminating lockon whn we're in auton
		if (shouldAutomaticallyEnd && isWithinTolerance()) {
			return true;
		}

		// Runs while button is held
		return false;
	}
}
