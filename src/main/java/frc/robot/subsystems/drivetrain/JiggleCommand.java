// package frc.robot.subsystems.drivetrain;

// import java.util.Collections;
// import java.util.Set;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// import frc.robot.subsystems.drivetrain.DrivetrainConstants.LockOnDriveConstraints;
// import frc.robot.subsystems.intake.IntakeConstants;
// import frc.robot.subsystems.shooter.LaunchCalculator;
// import frc.robot.util.TunableNumber;
// import frc.robot.util.Util4828;

// public class JiggleCommand extends Command{
//     // Tunable PID constants and constraints for jiggle behavior
// 	private static TunableNumber kP = new TunableNumber("Tuning/Jiggle/PID_P", LockOnDriveConstraints.kP);
// 	private static TunableNumber kI = new TunableNumber("Tuning/Jiggle/PID_I", LockOnDriveConstraints.kI);
// 	private static TunableNumber kD = new TunableNumber("Tuning/Jiggle/PID_D", LockOnDriveConstraints.kD);
// 	// Tunable constraints for the profiled PID controller
// 	private static TunableNumber maxAngularVelocity = new TunableNumber("Tuning/Jiggle/MaxRotVelo", LockOnDriveConstraints.MAX_ROTATIONAL_VELOCITY);
// 	private static TunableNumber maxAngularAcceleration = new TunableNumber("Tuning/Jiggle/MaxRotAccel", LockOnDriveConstraints.MAX_ROTATIONAL_ACCELERATION);
// 	private static TunableNumber aimToleranceDeg = new TunableNumber("Tuning/Jiggle/Tolerance", LockOnDriveConstraints.AIM_TOLERANCE_DEGREES);

//     private final CommandSwerveDrivetrain drivetrain;
// 	/** Driver translation input (field-relative) */
// 	private final CommandXboxController controller;

//     private final boolean shouldAutomaticallyEnd;

// 	/** Profiled PID Controller for rotation */
// 	private ProfiledPIDController headingPID;

//     private final SwerveRequest.FieldCentric driveRequest =
// 		new SwerveRequest.FieldCentric()
// 			.withDeadband(DrivetrainConstants.MAX_SPEED * DrivetrainConstants.DEADBAND)
// 			.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * DrivetrainConstants.ROTATIONAL_DEADBAND)
//       		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


//     //RightTurnJiggle
//     public Command changeRightAngle() {
//         return Commands.defer(
//             () -> {
//                 return Commands.run(() -> 
//                     {
//                         Pose2d robotPose = drivetrain.getState().Pose;
//                         Rotation2d currentHeading = robotPose.getRotation();

//                         Rotation2d rightRotation = Rotation2d.fromDegrees(DrivetrainConstants.JIGGLE_ANGLE);

//                         Rotation2d rightDesiredHeading = currentHeading.plus(rightRotation);

//                         double rightOmega = headingPID.calculate(currentHeading.getRadians(), rightDesiredHeading.getRadians());
//                         drivetrain.setControl(
//                             driveRequest
//                                 .withVelocityX(-controller.getLeftY() * DrivetrainConstants.MAX_SPEED * DrivetrainConstants.SPEED_LIMIT_MULTIPLIER)
//                                 .withVelocityY(-controller.getLeftX() * DrivetrainConstants.MAX_SPEED * DrivetrainConstants.SPEED_LIMIT_MULTIPLIER)
//                                 .withRotationalRate(rightOmega));
//                                     }
//                 );
//             },
//             Collections.emptySet()
//         );
//     }

//     //LeftTurnJiggle
//     public Command changeLefttAngle() {
//         return Commands.defer(
//             () -> {
//                 return Commands.run(() -> 
//                     {
//                         Pose2d robotPose = drivetrain.getState().Pose;
//                         Rotation2d currentHeading = robotPose.getRotation();

//                         Rotation2d leftRotation = Rotation2d.fromDegrees(-DrivetrainConstants.JIGGLE_ANGLE);

//                         Rotation2d leftDesiredHeading = currentHeading.plus(leftRotation);

//                         double leftOmega = headingPID.calculate(currentHeading.getRadians(), leftDesiredHeading.getRadians());
//                         drivetrain.setControl(
//                             driveRequest
//                                 .withVelocityX(-controller.getLeftY() * DrivetrainConstants.MAX_SPEED * DrivetrainConstants.SPEED_LIMIT_MULTIPLIER)
//                                 .withVelocityY(-controller.getLeftX() * DrivetrainConstants.MAX_SPEED * DrivetrainConstants.SPEED_LIMIT_MULTIPLIER)
//                                 .withRotationalRate(leftOmega));
//                                     }
//                 );
//             },
//             Collections.emptySet()
//         );
//     }

// 	public JiggleCommand(
// 		CommandSwerveDrivetrain drivetrain,
// 		CommandXboxController controller,
// 		boolean shouldAutomaticallyEnd
// 	) {
//         this.drivetrain = drivetrain;
// 		this.controller = controller;
//         this.shouldAutomaticallyEnd = shouldAutomaticallyEnd;

//         Pose2d robotPose = drivetrain.getState().Pose;

// 		addRequirements(drivetrain);
//     }

//     @Override
//     public void initialize() {
// 		// Create a trapezoid-profiled PID controller
// 		this.headingPID = new ProfiledPIDController(
// 			kP.get(),
// 			kI.get(),
// 			kD.get(),
// 			new TrapezoidProfile.Constraints(
// 				maxAngularVelocity.get(),
// 				maxAngularAcceleration.get())
// 		);
		
// 		// Enable wraparound for circular heading
// 		this.headingPID.enableContinuousInput(-Math.PI, Math.PI);
// 		this.headingPID.setTolerance(Math.toRadians(aimToleranceDeg.get()));

//         Pose2d robotPose = drivetrain.getState().Pose;
//         headingPID.reset(robotPose.getRotation().getRadians());
//     }

//     @Override
// 	public void execute() {
//         Commands.waitSeconds(DrivetrainConstants.JIGGLE_DELAY)
//             .andThen(Commands.repeatingSequence(
//                 changeRightAngle().withTimeout(DrivetrainConstants.JIGGLE_DELAY),
//                 changeLefttAngle().withTimeout(DrivetrainConstants.JIGGLE_DELAY)
//             )
//         );
// 	}

//     @Override
// 	public void end(boolean interrupted) {
// 		// Stop motion cleanly
// 		drivetrain.setControl(
// 			driveRequest
		
// 				.withVelocityX(0.0)
// 				.withVelocityY(0.0)
// 				.withRotationalRate(0.0)
// 		);
// 	}

//     @Override
// 	public boolean isFinished() {
// 		// Automatically terminating jiggle when we're in auton
// 		if (shouldAutomaticallyEnd) {
// 			return true;
// 		}

// 		// Runs while button is held so never finishes on its own during teleop
// 		return false;
// 	}
// }
