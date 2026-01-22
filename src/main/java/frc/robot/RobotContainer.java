// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
  /*** DRIVETRAIN SUBSYSTEM ***/
  public final CommandSwerveDrivetrain m_drivetrain;
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /*** SHOOTER SUBSYSTEM ***/
  private final Shooter m_shooter;

  /*** CLIMBER SUBSYSTEM ***/
  private final Climber m_climber;

  /*** INPUT DEVICES ***/
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain = TunerConstants.createDrivetrain();

    m_shooter = new Shooter();
    
    m_climber = new Climber();
    
    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    /*** DRIVETRAIN ***/
    // Default command for drivetrain - drive according to driver controller joystick
    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(() -> drive
            .withVelocityX(m_driverController.getLeftY() * MaxSpeed) // Drive forward with positive Y (forward)
            .withVelocityY(m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    
    // While disabled, idle.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );
    
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_driverController.back().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.back().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController.start().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController.start().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    m_driverController.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

    
    m_driverController.povUp().onTrue(new InstantCommand(() -> SignalLogger.start()));
    m_driverController.povDown().onTrue(new InstantCommand(() -> SignalLogger.stop()));

    /*** SHOOTER ***/
    m_driverController.a().onTrue(m_shooter.shoot());
    m_driverController.b().onTrue(m_shooter.stop());

    /*** CLIMBER ***/
    m_driverController.x().onTrue(m_climber.climbUp());
    m_driverController.x().onFalse(m_climber.stop());
    m_driverController.y().onTrue(m_climber.climbDown());
    m_driverController.y().onFalse(m_climber.stop());
  }
}
