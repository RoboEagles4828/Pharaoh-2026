package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Util4828;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoAlignToTowerCommand extends Command {
    private Command internalCommand = null;
    private boolean canceled = false;
    private CommandSwerveDrivetrain drive = null;

    public AutoAlignToTowerCommand(CommandSwerveDrivetrain drivetrain) {
        drive = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        //Auto Align values
        double targetX = SmartDashboard.getNumber(CommandSwerveDrivetrain.NT_TOWERALIGN_X, DrivetrainConstants.TOWER_ALIGN_X);
        double targetY = SmartDashboard.getNumber(CommandSwerveDrivetrain.NT_TOWERALIGN_Y, DrivetrainConstants.TOWER_ALIGN_Y);
        double targetTheta = SmartDashboard.getNumber(CommandSwerveDrivetrain.NT_TOWERALIGN_THETA, DrivetrainConstants.TOWER_ALIGN_THETA);

        Pose2d scoringPose = new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetTheta));

        double robotX = drive.getState().Pose.getX();
        double robotY = drive.getState().Pose.getY();

        double distanceFromTarget = Util4828.getDistance(robotX, robotY, targetX, targetY);
        
        if(distanceFromTarget > DrivetrainConstants.MAX_AUTOALIGN_TOWER_DISTANCE)
        {
            System.out.println("Aborted tower auto align, robot was too far away.");
            canceled = true;
            return;
        }

        // PathPlanner constraints (tune these!)
        PathConstraints constraints = new PathConstraints(
                0.5,  // max velocity (m/s)
                0.5,  // max accel (m/s^2)
                1.0,  // max angular vel (rad/s)
                1.0   // max angular accel
        );

        // Generate the PP pathfinding command
        internalCommand = AutoBuilder.pathfindToPose(scoringPose, constraints);

        internalCommand.initialize();
    }

    @Override
    public void execute() {
        if (internalCommand != null) {
            internalCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (internalCommand != null) {
            internalCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        if (canceled) {
            return true;
        }
        return internalCommand != null && internalCommand.isFinished();
    }
}