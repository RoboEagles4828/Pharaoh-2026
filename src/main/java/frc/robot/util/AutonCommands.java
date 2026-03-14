package frc.robot.util;

import java.util.Collections;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.LockOnDriveCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.Constants;

public class AutonCommands {

    /** Returns a process that will aim the robot and shoot */
    // TODO test with faster times
    public static Command aimAndShoot(
        CommandSwerveDrivetrain drivetrain,
        CommandXboxController driverController,
        Shooter shooter,
        Hopper hopper,
        Kicker kicker,
        LaunchCalculator launchCalculator
    ) {
        return 
        Commands.sequence(
            new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Locking on Hub")),
            (new LockOnDriveCommand(drivetrain, driverController, false, launchCalculator)).withTimeout(1.0),
            
            new ParallelDeadlineGroup(
                Commands.waitSeconds(2.5),
                new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Shoot")),
                shooter.raiseHood(),
                shooter.start(),
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    Commands.parallel(
                        hopper.startConveyor().withTimeout(2.0),
                        kicker.start().withTimeout(2.0)
                    )
                )
            )
        ).andThen(
            Commands.parallel(
                shooter.stop(),
                kicker.stop(),
                hopper.stopConveyor(),
                shooter.lowerHood()
            )
        );
    }

    /** Returns a command to climb the left of the tower (depot side of the field) */
    // TODO needs to be tested and tuned
    public static Command climbLeft(
        CommandSwerveDrivetrain drivetrain,
        Climber climber
    ) {
        return
        Commands.defer(() -> 
            Commands.sequence(
                new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Staging to tower LEFT")),
                drivetrain.stageToTower(Constants.FieldConstants.TowerSide.LEFT),

                new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Raising Climber")),
                climber.extendToPeak(),
                Commands.waitSeconds(0.8),

                new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Aligning to Tower")),
                drivetrain.alignToTower(),
                Commands.waitSeconds(1.0),

                new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Climbing Up")),
                climber.retractForClimb(),
                
                new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Climb Completed"))
            ),
            Collections.emptySet()
        );
    }

    /** Returns a command to climb the right side of the tower (human player side) */
    // TODO needs to be tested and tuned
    public static Command climbRight(
        CommandSwerveDrivetrain drivetrain,
        Climber climber
    ) {
        return
        Commands.defer(() -> 
            Commands.sequence(
                new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Staging to tower RIGHT")),
                drivetrain.stageToTower(Constants.FieldConstants.TowerSide.RIGHT),

                new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Raising Climber")),
                climber.extendToPeak(),
                Commands.waitSeconds(0.8),

                new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Aligning to Tower")),
                drivetrain.alignToTower(),
                Commands.waitSeconds(1.0),

                new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Climbing Up")),
                climber.retractForClimb(),
                
                new InstantCommand(() -> SmartDashboard.putString("AutonStage", "Climb Completed"))
            ),
            Collections.emptySet()
        );
    }
}
