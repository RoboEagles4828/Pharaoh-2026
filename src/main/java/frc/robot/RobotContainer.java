// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
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
import frc.robot.util.AutonCommands;
import frc.robot.util.PoseSupplier;
import frc.robot.util.TunableNumber;

import java.util.Collections;

import com.pathplanner.lib.auto.AutoBuilder;

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
  @SuppressWarnings("unused")
  private Vision vision = null;

  /*** POSE SUPPLIER ***/
  private PoseSupplier poseSupplier = null;

  /*** LAUNCH CALCULATOR */
  private LaunchCalculator launchCalculator = null;

  /*** INPUT DEVICES ***/
  private CommandXboxController driverController;

  /*** PATHPLANNER WIDGET ***/
  private final SendableChooser<Command> autonomousChooser;


  private static final TunableNumber autonDelay = new TunableNumber("Tuning/Auton/AutonDelay", Constants.AUTON_DELAY);

  /*** more complex commands that require multiple subsystems */
  // public Command aimAndShoot() {
  //   return Commands.sequence(
  //     Commands.print("Start aim and shoot"),
  //     (new LockOnDriveCommand(drivetrain, driverController, true)).withTimeout(1.0),

  //     new ParallelDeadlineGroup(
  //       Commands.waitSeconds(3.0),
  //       Commands.print("Aim and shoot"),
  //       shooter.raiseHood(),
  //       shooter.start(),
  //       Commands.sequence(
  //         Commands.waitSeconds(1.0),
  //         Commands.parallel(
  //           hopper.startConveyor().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withTimeout(2.0),
  //           kicker.start().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withTimeout(2.0)
  //         )
  //       )
  //     )
  //   ).andThen(
  //     Commands.parallel(
  //       shooter.stop(),
  //       kicker.stop(),
  //       hopper.stopConveyor(),
  //       shooter.lowerHood()
  //     )
  //   );

  // }

  // public Command climbLeft() {
  //   return Commands.defer(() -> 
  //     Commands.sequence(
  //       Commands.print("Staging to tower LEFT."),
  //       drivetrain.stageToTower(Constants.FieldConstants.TowerSide.LEFT),
  //       Commands.print("Raising climber."),
  //       climber.extendToPeak(),
  //       Commands.waitSeconds(0.8),
  //       Commands.print("Aligning to tower."),
  //       drivetrain.alignToTower(),
  //       Commands.waitSeconds(1.0),
  //       Commands.print("Climbing up."),
  //       climber.retractForClimb(),
  //       Commands.print("Climb completed.")
  //       ), Collections.emptySet());
  // }

  // public Command climbRight() {
  //   return Commands.defer(() -> 
  //     Commands.sequence(
  //       Commands.print("Staging to tower RIGHT."),
  //       drivetrain.stageToTower(Constants.FieldConstants.TowerSide.RIGHT),
  //       Commands.print("Raising climber."),
  //       climber.extendToPeak(),
  //       Commands.waitSeconds(0.5),
  //       Commands.print("Aligning to tower."),
  //       drivetrain.alignToTower(),
  //       Commands.waitSeconds(1.0),
  //       Commands.print("Climbing up."),
  //       //climber.retractForClimb(),
  //       Commands.print("Climb completed.")
  //       ), Collections.emptySet());
  // }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // For 2026, the field is rotationally symmetrical, not mirrored.
    // We need to tell PathPlanner this so it correctly rotates our
    //  auto paths for the red alliance side. (By default, it mirrors, we want 180 rotation.)
    FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;

    /** SUBSYSTEMS **/
    drivetrain = TunerConstants.createDrivetrain();
    poseSupplier = new PoseSupplier(drivetrain);

    launchCalculator = new LaunchCalculator(poseSupplier, drivetrain);
    shooter = new Shooter(launchCalculator);
    
    kicker = new Kicker();

    intake = new Intake();
  
    hopper = new Hopper();

    climber = new Climber();

    vision = new Vision(drivetrain, poseSupplier);
    
    driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /** PATHPLANNER **/
    NamedCommands.registerCommand("Shoot", Commands.defer(() -> AutonCommands.aimAndShoot(drivetrain, driverController, shooter, hopper, kicker, launchCalculator), Collections.emptySet()));
    NamedCommands.registerCommand("ClimbRight", Commands.defer(() -> AutonCommands.climbRight(drivetrain, climber), Collections.emptySet()));
    NamedCommands.registerCommand("ClimbLeft", Commands.defer(() -> AutonCommands.climbLeft(drivetrain, climber), Collections.emptySet()));
		NamedCommands.registerCommand("StartIntake", intake.intake());
		NamedCommands.registerCommand("StopIntake", intake.stopAndRetract().withTimeout(1.0));
    NamedCommands.registerCommand("StopIntakeWheels", AutonCommands.stopIntaking(intake));

		// Create and populate a SendableChooser with the autonomous routines from PathPlanner, and add it to dashboard.
		autonomousChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autonomousChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

    /*** Driving */
    // Default command for drivetrain - drive according to driver controller joystick
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> driveRequest
            .withVelocityX(-driverController.getLeftY() * DrivetrainConstants.MAX_SPEED) 
            .withVelocityY(-driverController.getLeftX() * DrivetrainConstants.MAX_SPEED) 
            .withRotationalRate(-driverController.getRightX() * DrivetrainConstants.MAX_ANGULAR_RATE) 
        ));

    // While disabled, idle.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Run SysId routines when holding back/start and X/Y. Note that each routine should be run exactly once in a single log.
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Use dpad for basic movement in the 4 cardinal directions
    // driverController.povUp().whileTrue(drivetrain.applyRequest(() -> driveRequestRobotCentric.withVelocityX(0.1 * DrivetrainConstants.MAX_SPEED).withVelocityY(0.0).withRotationalRate(0.0)));
    // driverController.povDown().whileTrue(drivetrain.applyRequest(() -> driveRequestRobotCentric.withVelocityX(-0.1 * DrivetrainConstants.MAX_SPEED).withVelocityY(0.0).withRotationalRate(0.0)));
    // driverController.povRight().whileTrue(drivetrain.applyRequest(() -> driveRequestRobotCentric.withVelocityX(0.0).withVelocityY(-0.1 * DrivetrainConstants.MAX_SPEED).withRotationalRate(0)));
    // driverController.povLeft().whileTrue(drivetrain.applyRequest(() -> driveRequestRobotCentric.withVelocityX(0.0).withVelocityY(0.1 * DrivetrainConstants.MAX_SPEED).withRotationalRate(0)));

    // Reset the field-centric heading
    // driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    driverController.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    /*** Intaking */
    driverController.leftTrigger().whileTrue(intake.intake());
    driverController.leftTrigger().whileTrue(hopper.startConveyor());
    //Ben - At Raza's request, make it possible to intake and shoot at the same time.
    //This shouldn't actually change our ball capacity. kicker.startIntake and shooter.startIntake both
    //don't really achieve anything (shooter spins backwards and kicker doesn't spin at all).
    //driverController.leftTrigger().whileTrue(shooter.startIntake());
    //driverController.leftTrigger().whileTrue(kicker.startIntake());

    driverController.b().whileTrue(intake.outtake());

    /*** Aim/lockon **/
    driverController.leftBumper().whileTrue(
      Commands.defer(() -> {
        return new LockOnDriveCommand(drivetrain, driverController,false, launchCalculator);
        }, Collections.emptySet()
      )
    );
    driverController.leftBumper().whileTrue(shooter.start());
    driverController.leftBumper().whileTrue(shooter.raiseHood());
    driverController.leftBumper().onFalse(shooter.lowerHood()); //< this is sort a $hack$ but it's ok for now...

    Trigger readyToShoot = new Trigger(() -> 
      shooter.isAtTargetParams() && driverController.leftBumper().getAsBoolean());

    readyToShoot.onTrue(
      Commands.sequence(
        new InstantCommand(() -> driverController.setRumble(RumbleType.kBothRumble, 0.5)),
        Commands.waitSeconds(0.5),
        new InstantCommand(() -> driverController.setRumble(RumbleType.kBothRumble, 0))
      )
    );

    /*** Shooting */
    driverController.rightTrigger().whileTrue(hopper.startConveyor());
    driverController.rightTrigger().whileTrue(kicker.start());

    driverController.x().onTrue(Commands.runOnce(() -> launchCalculator.enterHubShotMode()));
    driverController.x().onFalse(Commands.runOnce(() -> launchCalculator.enterShootFromAnywhereMode()));
    driverController.rightBumper().onTrue(Commands.runOnce(() -> launchCalculator.enterFarShotMode()));
    driverController.rightBumper().onFalse(Commands.runOnce(() -> launchCalculator.enterShootFromAnywhereMode()));

    /*** Climbing ***/
    //driverController.b().onTrue(climbRight());
    //driverController.x().onTrue(climbLeft());
    driverController.y().onTrue(climber.extendToPeak());
    driverController.a().onTrue(climber.retractToBottom());

    //driverController.rightBumper().whileTrue(climber.climbDownDutyCycle());
    //driverController.rightBumper().onFalse(climber.stop());
  }

  public void setDefaultCommands() {
    // Set defaults after the autonomous performs so it doesn't interfere with the command
    shooter.setDefaultCommand(shooter.stop());
    kicker.setDefaultCommand(kicker.stop());
    intake.setDefaultCommand(intake.stopAndRetract());
    hopper.setDefaultCommand(hopper.stopConveyor());
  }

  public Command getAutonomousCommand() {
    return Commands.defer(
      () -> {
        return Commands.sequence(
          Commands.waitSeconds(autonDelay.get()),
          autonomousChooser.getSelected()
        );
      },
      Collections.emptySet()
    );
  }
}
