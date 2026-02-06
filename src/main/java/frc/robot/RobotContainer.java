// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Util4828;

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
  private static final boolean ENABLE_DRIVETRAIN = true;
  private static final boolean ENABLE_SHOOTER = false;
  private static final boolean ENABLE_INTAKE = false;
  private static final boolean ENABLE_LIMELIGHT = true;
  private static final boolean ENABLE_CLIMBER = false;


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

  /*** INTAKE SUBSYSTEM ***/
  private Intake intake = null;

  /*** CLIMBER SUBSYSTEM ***/
  private Climber climber = null;

  /*** LIMELIGHT SUBSYSTEM ***/
  private Limelight limelight = null;

  /*** INPUT DEVICES ***/
  private CommandXboxController driverController;

  /*** PATHPLANNER WIDGET ***/
  private final SendableChooser<Command> autonomousChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (ENABLE_DRIVETRAIN)
      drivetrain = TunerConstants.createDrivetrain();

    if (ENABLE_SHOOTER)
      shooter = new Shooter();

    if (ENABLE_INTAKE)
      intake = new Intake();

    if (ENABLE_CLIMBER)
      climber = new Climber();

    if (ENABLE_LIMELIGHT)
      limelight = new Limelight(drivetrain);
    
    driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // Pathplanner
    // TODO - register commands here
    // NamedCommands.registerCommand("CommandName", command);
    autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Chooser", autonomousChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    /*** DRIVETRAIN ***/
    if (drivetrain != null) {
      // Default command for drivetrain - drive according to driver controller joystick
      drivetrain.setDefaultCommand(
          drivetrain.applyRequest(() -> driveRequest
              .withVelocityX(driverController.getLeftY() * DrivetrainConstants.MAX_SPEED) // Drive forward with positive Y (forward)
              .withVelocityY(driverController.getLeftX() * DrivetrainConstants.MAX_SPEED) // Drive left with negative X (left)
              .withRotationalRate(-driverController.getRightX() * DrivetrainConstants.MAX_ANGULAR_RATE) // Drive counterclockwise with negative X (left)
          ));

      // While disabled, idle.
      final var idle = new SwerveRequest.Idle();
      RobotModeTriggers.disabled().whileTrue(
          drivetrain.applyRequest(() -> idle).ignoringDisable(true)
      );

      // Lock-on to HUB while holding right trigger
      driverController.rightBumper().whileTrue(
        new LockOnDriveCommand(
          drivetrain,
          driverController,
          Util4828.getHubLocation()
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

      // Reset the field-centric heading on left bumper press.
      driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

      driverController.povUp().onTrue(new InstantCommand(() -> SignalLogger.start()));
      driverController.povDown().onTrue(new InstantCommand(() -> SignalLogger.stop()));

      driverController.leftTrigger().onTrue(drivetrain.alignToTower(Constants.FieldConstants.TowerSide.LEFT));
      driverController.rightTrigger().onTrue(drivetrain.alignToTower(Constants.FieldConstants.TowerSide.RIGHT));
    }

    /*** SHOOTER ***/
    if (shooter != null) {
      driverController.a().onTrue(shooter.start());
      driverController.b().onTrue(shooter.stop());
    }

    /*** CLIMBER ***/
    if (climber != null) {
      climber.setDefaultCommand(climber.stop());
      driverController.x().whileTrue(climber.climbUp());
      driverController.y().whileTrue(climber.climbDown());
    }

    /*** INTAKE ***/
    if (intake != null) {
      intake.setDefaultCommand(intake.stop());
      driverController.leftBumper().whileTrue(intake.start());
    }
  }
}
