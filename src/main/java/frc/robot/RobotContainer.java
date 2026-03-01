// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.LockOnDriveCommand;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.PoseSupplier;
import frc.robot.util.Util4828;

import java.util.Collections;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /*** Flags which control which subsystems are instantiated. ***/
  private static final boolean ENABLE_SHOOTER = true;
  private static final boolean ENABLE_KICKER = true;
  private static final boolean ENABLE_INTAKE = true;
  private static final boolean ENABLE_HOPPER = true;
  private static final boolean ENABLE_VISION = true;
  private static final boolean ENABLE_CLIMBER = true;


  /*** DRIVETRAIN SUBSYSTEM ***/
  public CommandSwerveDrivetrain drivetrain = null;
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(DrivetrainConstants.MAX_SPEED * DrivetrainConstants.DEADBAND)
      .withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * DrivetrainConstants.ROTATIONAL_DEADBAND) 
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.RobotCentric driveRequestRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(DrivetrainConstants.MAX_SPEED * DrivetrainConstants.DEADBAND)
      .withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * DrivetrainConstants.ROTATIONAL_DEADBAND) 
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


  /*** SHOOTER SUBSYSTEM ***/
  private Shooter shooter = null;

  /*** KICKER SUBSYSTEM ***/
  private Kicker kicker = null;

  /*** INTAKE SUBSYSTEM ***/
  private Intake intake = null;

  /*** HOPPER SUBSYSTEM */
  private Hopper hopper = null;

  /*** CLIMBER SUBSYSTEM ***/
  private Climber climber = null;

  /*** VISION SUBSYSTEM ***/
  private Vision vision = null;

  /*** POSE SUPPLIER ***/
  private PoseSupplier poseSupplier = null;

  /*** LAUNCH CALCULATOR */
  private LaunchCalculator launchCalculator = null;

  /*** INPUT DEVICES ***/
  private CommandXboxController driverController;

  /* === COMMANDS === */
	// Register named commands
	private final Command shootCommand = new InstantCommand(() -> shooter.start());
	private final Command autonLockOnCommand = new LockOnDriveCommand(drivetrain, driverController, true);
	private final Command climbCommand = new InstantCommand(() -> climber.climb());
	private final Command intakeCommand = new InstantCommand(() -> intake.intake());

  /*** PATHPLANNER WIDGET ***/
  //private final SendableChooser<Command> autonomousChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain = TunerConstants.createDrivetrain();
    poseSupplier = new PoseSupplier(drivetrain);

    if (ENABLE_SHOOTER){
      launchCalculator = new LaunchCalculator(poseSupplier);
      shooter = new Shooter(launchCalculator);
    }

    if (ENABLE_KICKER)
      kicker = new Kicker();

    if (ENABLE_INTAKE)
      intake = new Intake();
    
    if (ENABLE_HOPPER)
      hopper = new Hopper();

    if (ENABLE_CLIMBER)
      climber = new Climber();

    if (ENABLE_VISION)
      vision = new Vision(drivetrain, poseSupplier);
    
    driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // Pathplanner
    NamedCommands.registerCommand("Shoot", shootCommand);
		NamedCommands.registerCommand("LockOnHub", autonLockOnCommand);
		NamedCommands.registerCommand("ClimbL1", climbCommand);
		NamedCommands.registerCommand("Intake", intakeCommand);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    /*** DRIVETRAIN ***/
    if (drivetrain != null) {
      // Default command for drivetrain - drive according to driver controller joystick
      drivetrain.setDefaultCommand(
          drivetrain.applyRequest(() -> driveRequest
              .withVelocityX(-driverController.getLeftY() * DrivetrainConstants.MAX_SPEED) // Drive forward with positive Y (forward)
              .withVelocityY(-driverController.getLeftX() * DrivetrainConstants.MAX_SPEED) // Drive left with negative X (left)
              .withRotationalRate(-driverController.getRightX() * DrivetrainConstants.MAX_ANGULAR_RATE) // Drive counterclockwise with negative X (left)
          ));

      // While disabled, idle.
      final var idle = new SwerveRequest.Idle();
      RobotModeTriggers.disabled().whileTrue(
          drivetrain.applyRequest(() -> idle).ignoringDisable(true)
      );
      
      driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

      // Lock-on while holding right trigger
      driverController.leftBumper().whileTrue(
        Commands.defer(() -> {
          return new LockOnDriveCommand(
            drivetrain,
            driverController,
            false
          );
        }, Collections.emptySet()
        )

      );

      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

      // Use dpad for basic movement in the 4 cardinal directions
      // Drive straight forward slowly
      driverController.povUp().whileTrue(
        drivetrain.applyRequest(() -> driveRequestRobotCentric
          .withVelocityX(0.1 * DrivetrainConstants.MAX_SPEED)
          .withVelocityY(0.0)
          .withRotationalRate(0.0)));
      // Drive straight backward slowly
      driverController.povDown().whileTrue(
        drivetrain.applyRequest(() -> driveRequestRobotCentric
          .withVelocityX(-0.1 * DrivetrainConstants.MAX_SPEED)
          .withVelocityY(0.0)
          .withRotationalRate(0.0)));
      // Drive straight right slowly
      driverController.povRight().whileTrue(
        drivetrain.applyRequest(() -> driveRequestRobotCentric
          .withVelocityX(0.0)
          .withVelocityY(-0.1 * DrivetrainConstants.MAX_SPEED)
          .withRotationalRate(0)));
      // Drive straight left slowly
      driverController.povLeft().whileTrue(
        drivetrain.applyRequest(() -> driveRequestRobotCentric
          .withVelocityX(0.0)
          .withVelocityY(0.1 * DrivetrainConstants.MAX_SPEED)
          .withRotationalRate(0)));

      // Reset the field-centric heading on start
      driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

      //driverController.povUp().onTrue(new InstantCommand(() -> SignalLogger.start()));
      //driverController.povDown().onTrue(new InstantCommand(() -> SignalLogger.stop()));

      driverController.a().onTrue(drivetrain.alignToTower(Constants.FieldConstants.TowerSide.LEFT));
      driverController.b().onTrue(drivetrain.alignToTower(Constants.FieldConstants.TowerSide.RIGHT));
      
    }

    /*** SHOOTER ***/
    if (shooter != null) {
      shooter.setDefaultCommand(shooter.stop());
      driverController.leftBumper().whileTrue(shooter.start());

      driverController.leftBumper().whileTrue(shooter.raiseHood());
      driverController.leftBumper().whileFalse(shooter.lowerHood());
    }

    /*** KICKER ***/
    if (kicker != null) {
      kicker.setDefaultCommand(kicker.stop());
    }

    /*** CLIMBER ***/
    if (climber != null) {
      climber.setDefaultCommand(climber.stop());
      driverController.y().whileTrue(climber.climbUpDutyCycle());
      driverController.x().whileTrue(climber.climbDownDutyCycle());
      // driverController.y().onTrue(climber.climbToPeak());
      // driverController.x().onTrue(climber.retractClimb());
    }

    /*** INTAKE ***/
    if (intake != null) {
      intake.setDefaultCommand(intake.stopAndRetract());
      driverController.leftTrigger().whileTrue(intake.intake());

      // Also run the conveyor while intaking
      if (hopper != null) {
        driverController.leftTrigger().whileTrue(hopper.startConveyor());
      }
      if (shooter != null) {
        driverController.leftTrigger().whileTrue(shooter.startIntake());
      }
      if (kicker != null) {
        driverController.leftTrigger().whileTrue(kicker.startIntake());
      }
    }

    /*** HOPPER ***/
    if (hopper != null) {
      hopper.setDefaultCommand(hopper.stopConveyor());
      driverController.rightTrigger().whileTrue(hopper.startConveyor());

      // We're trying to shoot, start kicker
      if (kicker != null) {
        driverController.rightTrigger().whileTrue(kicker.start());
      }
    }
  }
}
