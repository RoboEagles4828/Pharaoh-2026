package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Util4828;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoAlignToTowerCommand extends Command {
    enum Step {
        ONE, TWO, FINISHED
    }

    private Command ppCommand = null;
    private boolean canceled = false;
    private CommandSwerveDrivetrain drive = null;
    private Step step;

    public AutoAlignToTowerCommand(CommandSwerveDrivetrain drivetrain) {
        drive = drivetrain;
        addRequirements(drivetrain);
    }

    private Command getPPStep1Command() {
        return null; // todo
    }

    private Command getPPStep2Command() {
        return null; // todo
    }

    @Override
    public void initialize() {
        step = Step.ONE;

        //Auto Align values
        double targetX = SmartDashboard.getNumber(CommandSwerveDrivetrain.NT_TOWERALIGN_STEP1_X, DrivetrainConstants.TOWER_ALIGN_STEP1_X);
        double targetY = SmartDashboard.getNumber(CommandSwerveDrivetrain.NT_TOWERALIGN_STEP1_Y, DrivetrainConstants.TOWER_ALIGN_STEP1_Y);
        double targetTheta = SmartDashboard.getNumber(CommandSwerveDrivetrain.NT_TOWERALIGN_STEP1_THETA, DrivetrainConstants.TOWER_ALIGN_STEP1_THETA);

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
        ppCommand = AutoBuilder.pathfindToPose(scoringPose, constraints);

        ppCommand.initialize();
    }

    @Override
    public void execute() {
        // something has gone horribly wrong; abort the command
        if (ppCommand == null) {
            step = Step.FINISHED;
            return;
        }
        
        // If one of our pathplanner commands is done, either move to the next step
        // or, if the last step is done, note that we're finished.
        if (ppCommand.isFinished()) {
            if (step == Step.ONE) {
                ppCommand = getPPStep2Command();
                step = Step.TWO;
            }
            else if (step == Step.TWO) {
                ppCommand = null;
                step = Step.FINISHED;
            }
        }

        // run the pathplanner command
        if (ppCommand != null) {
            ppCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (ppCommand != null) {
            ppCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        if (canceled) {
            return true;
        }
        return step == Step.FINISHED;
    }
}