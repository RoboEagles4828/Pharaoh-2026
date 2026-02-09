package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

public class PIDSwerve extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;
    private final PIDController xController, yController, rotationController;
    private final SwerveRequest.FieldCentric driveRequest;
    private final Timer alignedTimer = new Timer();

    public PIDSwerve(
        CommandSwerveDrivetrain drivetrain,
        Pose2d targetPose
    ) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.xController = new PIDController(
            PIDAutoAlignConstants.X_PID_CONSTANTS.kP,
            PIDAutoAlignConstants.X_PID_CONSTANTS.kI,
            PIDAutoAlignConstants.X_PID_CONSTANTS.kD);
        this.yController = new PIDController(
            PIDAutoAlignConstants.Y_PID_CONSTANTS.kP,
            PIDAutoAlignConstants.Y_PID_CONSTANTS.kI,
            PIDAutoAlignConstants.Y_PID_CONSTANTS.kD);
        this.rotationController = new PIDController(
            PIDAutoAlignConstants.ROTATION_PID_CONSTANTS.kP,
            PIDAutoAlignConstants.ROTATION_PID_CONSTANTS.kI,
            PIDAutoAlignConstants.ROTATION_PID_CONSTANTS.kD);
        driveRequest  = new SwerveRequest.FieldCentric()
            .withDeadband(DrivetrainConstants.MAX_SPEED * DrivetrainConstants.DEADBAND)
            .withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * DrivetrainConstants.ROTATIONAL_DEADBAND) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        rotationController.reset();
        alignedTimer.stop();
        alignedTimer.restart();


        xController.setIZone(PIDAutoAlignConstants.X_I_ZONE);
        xController.setIntegratorRange(-TunerConstants.driveGains.kS * 2, TunerConstants.driveGains.kS * 2);
        xController.setTolerance(PIDAutoAlignConstants.X_TOLERANCE, PIDAutoAlignConstants.POSITION_ERROR_DERIVATIVE_TOLERANCE);
        xController.setSetpoint(targetPose.getX());

        yController.setIZone(PIDAutoAlignConstants.Y_I_ZONE);
        yController.setIntegratorRange(-TunerConstants.driveGains.kS * 2, TunerConstants.driveGains.kS * 2);
        yController.setTolerance(PIDAutoAlignConstants.Y_TOLERANCE, PIDAutoAlignConstants.POSITION_ERROR_DERIVATIVE_TOLERANCE);
        yController.setSetpoint(targetPose.getY());

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setIZone(PIDAutoAlignConstants.ROTATION_I_ZONE);
        rotationController.setIntegratorRange(-TunerConstants.steerGains.kS * 2, TunerConstants.steerGains.kS * 2);
        rotationController.setTolerance(PIDAutoAlignConstants.ROTATION_TOLERANCE, PIDAutoAlignConstants.ROTATION_ERROR_DERIVATIVE_TOLERANCE);
        rotationController.setSetpoint(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        
        double xCorrection = xController.calculate(currentPose.getX());
        double xFeedforward = Math.copySign(TunerConstants.driveGains.kS, xCorrection);
        double xOutput = MathUtil.clamp(xCorrection + xFeedforward,
            -PIDAutoAlignConstants.MAX_SPEED, PIDAutoAlignConstants.MAX_SPEED);
        
        double yCorrection = yController.calculate(currentPose.getY());
        double yFeedforward = Math.copySign(TunerConstants.driveGains.kS, yCorrection);
        double yOutput = MathUtil.clamp(yCorrection + yFeedforward,
            -PIDAutoAlignConstants.MAX_SPEED, PIDAutoAlignConstants.MAX_SPEED);

        double rotationCorrection = rotationController.calculate(currentPose.getRotation().getRadians());
        double rotationFeedforward = Math.copySign(TunerConstants.steerGains.kS, rotationCorrection);
        double rotationOutput = MathUtil.clamp(rotationCorrection + rotationFeedforward,
            -PIDAutoAlignConstants.MAX_ANGULAR_SPEED, PIDAutoAlignConstants.MAX_ANGULAR_SPEED);

        drivetrain.applyRequest( () -> driveRequest
            .withVelocityX(xOutput)
            .withVelocityY(yOutput)
            .withRotationalRate(rotationOutput)
        );
    }

    @Override
    public boolean isFinished() {
        boolean xAligned = xController.atSetpoint();
        boolean yAligned = yController.atSetpoint();
        boolean rotationAligned = rotationController.atSetpoint();

        return (xAligned && yAligned && rotationAligned) || alignedTimer.hasElapsed(PIDAutoAlignConstants.TIMEOUT);
    }
}