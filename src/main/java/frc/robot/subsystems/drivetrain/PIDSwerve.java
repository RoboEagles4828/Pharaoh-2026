package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PIDSwerve extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPos;
    private PPHolonomicDriveController driveController = new PPHolonomicDriveController(
        DrivetrainConstants.PID_AUTOALIGN_DRIVE, DrivetrainConstants.PID_AUTOALIGN_ROTATE);
    
    public PIDSwerve(CommandSwerveDrivetrain drivetrain, Pose2d targetPos) {
        this.drivetrain = drivetrain;
        this.targetPos = targetPos;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();

        
    }
}
