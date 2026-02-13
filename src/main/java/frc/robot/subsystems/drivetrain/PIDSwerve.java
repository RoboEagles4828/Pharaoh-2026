package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.*;
import frc.robot.util.TunableNumber;

public class PIDSwerve extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private Pose2d targetPose;
    private PIDController xController, yController, rotationController;
    private final SwerveRequest.FieldCentric driveRequest;
    private final Timer alignedTimer = new Timer();

    public TunableNumber maxSpeed = new TunableNumber("Tuning/Drivetrain/PIDMaxSpeed", PIDAutoAlignConstants.MAX_SPEED);
    public TunableNumber maxAngularSpeed = new TunableNumber("Tuning/Drivetrain/PIDMaxAngularSpeed", PIDAutoAlignConstants.MAX_ANGULAR_SPEED);

    public static TunableNumber xP = new TunableNumber("Tuning/Drivetrain/X_P", 0.07);
    public static TunableNumber xI = new TunableNumber("Tuning/Drivetrain/X_I", 0.0);
    public static TunableNumber xD = new TunableNumber("Tuning/Drivetrain/X_D", 0.0);

    public static TunableNumber yP = new TunableNumber("Tuning/Drivetrain/Y_P", 0.07);
    public static TunableNumber yI = new TunableNumber("Tuning/Drivetrain/Y_I", 0.0);
    public static TunableNumber yD = new TunableNumber("Tuning/Drivetrain/Y_D", 0.0);

    public static TunableNumber rotationP = new TunableNumber("Tuning/Drivetrain/ROTATION_P", 0.015);
    public static TunableNumber rotationI = new TunableNumber("Tuning/Drivetrain/ROTATION_I", 0.0);
    public static TunableNumber rotationD = new TunableNumber("Tuning/Drivetrain/ROTATION_D", 0.0);

    TunableNumber targetX = new TunableNumber("Tuning/Drivetrain/TargetX", 0);
    TunableNumber targetY = new TunableNumber("Tuning/Drivetrain/TargetY", 0);
    TunableNumber targetRotation = new TunableNumber("Tuning/Drivetrain/TargetRotation", 0);

    public PIDSwerve(
        CommandSwerveDrivetrain drivetrain
        // Pose2d targetPose
    ) {
        this.drivetrain = drivetrain;
        this.targetPose = new Pose2d();
        // this.xController = new PIDController(
        //     PIDAutoAlignConstants.X_PID_CONSTANTS.kP,
        //     PIDAutoAlignConstants.X_PID_CONSTANTS.kI,
        //     PIDAutoAlignConstants.X_PID_CONSTANTS.kD);
        // this.yController = new PIDController(
        //     PIDAutoAlignConstants.Y_PID_CONSTANTS.kP,
        //     PIDAutoAlignConstants.Y_PID_CONSTANTS.kI,
        //     PIDAutoAlignConstants.Y_PID_CONSTANTS.kD);
        // this.rotationController = new PIDController(
        //     PIDAutoAlignConstants.ROTATION_PID_CONSTANTS.kP,
        //     PIDAutoAlignConstants.ROTATION_PID_CONSTANTS.kI,
        //     PIDAutoAlignConstants.ROTATION_PID_CONSTANTS.kD);
        this.xController = new PIDController(xP.get(), xI.get(), xD.get());
        this.yController = new PIDController(yP.get(), yI.get(), yD.get());
        this.rotationController = new PIDController(rotationP.get(), rotationI.get(), rotationD.get());

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

        targetPose = new Pose2d(targetX.get(), targetY.get(), Rotation2d.fromDegrees(targetRotation.get()));    
        xController = new PIDController(xP.get(), xI.get(), xD.get());
        yController = new PIDController(yP.get(), yI.get(), yD.get());
        rotationController = new PIDController(rotationP.get(), rotationI.get(), rotationD.get());

        xController.setIZone(PIDAutoAlignConstants.X_I_ZONE);
        xController.setIntegratorRange(-TunerConstants.driveGains.kS * 2, TunerConstants.driveGains.kS * 2);
        xController.setTolerance(PIDAutoAlignConstants.X_TOLERANCE, PIDAutoAlignConstants.POSITION_ERROR_DERIVATIVE_TOLERANCE);
        xController.setSetpoint(Units.metersToInches(targetPose.getX()));

        yController.setIZone(PIDAutoAlignConstants.Y_I_ZONE);
        yController.setIntegratorRange(-TunerConstants.driveGains.kS * 2, TunerConstants.driveGains.kS * 2);
        yController.setTolerance(PIDAutoAlignConstants.Y_TOLERANCE, PIDAutoAlignConstants.POSITION_ERROR_DERIVATIVE_TOLERANCE);
        yController.setSetpoint(Units.metersToInches(targetPose.getY()));

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setIZone(PIDAutoAlignConstants.ROTATION_I_ZONE);
        rotationController.setIntegratorRange(-TunerConstants.steerGains.kS * 2, TunerConstants.steerGains.kS * 2);
        rotationController.setTolerance(PIDAutoAlignConstants.ROTATION_TOLERANCE, PIDAutoAlignConstants.ROTATION_ERROR_DERIVATIVE_TOLERANCE);
        rotationController.setSetpoint(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        
        double xCorrection = xController.calculate(Units.metersToInches(currentPose.getX()));
        // double xFeedforward = Math.copySign(TunerConstants.driveGains.kS, xCorrection);
        double xOutput = MathUtil.clamp(xCorrection /*+ xFeedforward*/, -1.0, 1.0);
            // -maxSpeed.get(), maxSpeed.get());

        
        
        double yCorrection = yController.calculate(Units.metersToInches(currentPose.getY()));
        // double yFeedforward = Math.copySign(TunerConstants.driveGains.kS, yCorrection);
        double yOutput = MathUtil.clamp(yCorrection /*+ yFeedforward*/, -1.0, 1.0);
            // -maxSpeed.get(), maxSpeed.get());

        // Rotation2d direction = new Rotation2d(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());
        double rotationCorrection = rotationController.calculate(currentPose.getRotation().getRadians());
        // double rotationFeedforward = Math.copySign(TunerConstants.steerGains.kS, rotationCorrection);
        double rotationOutput = MathUtil.clamp(rotationCorrection /*+ rotationFeedforward*/, -1.0, 1.0);
            // -maxAngularSpeed.get(), maxAngularSpeed.get());

        drivetrain.applyRequest( () -> driveRequest
            .withVelocityX(xOutput * maxSpeed.get()) //* direction.getCos())
            .withVelocityY(yOutput * maxSpeed.get()) //* direction.getSin())
            .withRotationalRate(rotationOutput * maxAngularSpeed.get())
        ).execute();

        double[] target = {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()};
        SmartDashboard.putNumberArray("Target Pose", target);

        SmartDashboard.putNumber("xCorrection", xCorrection);
        SmartDashboard.putNumber("yCorrection", yCorrection);
        SmartDashboard.putNumber("rotationCorrection", rotationCorrection);

        // SmartDashboard.putNumber("xC+O", xCorrection + xFeedforward);
        // SmartDashboard.putNumber("yC+O", yCorrection + yFeedforward);
        // SmartDashboard.putNumber("rotationC+O", rotationCorrection + rotationFeedforward);

        SmartDashboard.putNumber("xOutput", xOutput);
        SmartDashboard.putNumber("yOutput", yOutput);
        SmartDashboard.putNumber("rotationOutput", rotationOutput);

        SmartDashboard.putNumber("Velocity X", xOutput * PIDAutoAlignConstants.MAX_SPEED /* * direction.getCos()*/);
        SmartDashboard.putNumber("Velocity Y", yOutput * PIDAutoAlignConstants.MAX_SPEED /* * direction.getSin()*/);
        SmartDashboard.putNumber("Rotational Rate", rotationOutput * PIDAutoAlignConstants.MAX_ANGULAR_SPEED);

        boolean xAligned = xController.atSetpoint();
        boolean yAligned = yController.atSetpoint();
        boolean rotationAligned = rotationController.atSetpoint();

        SmartDashboard.putBoolean("X Aligned", xAligned);
        SmartDashboard.putBoolean("Y Aligned", yAligned);
        SmartDashboard.putBoolean("Rotation Aligned", rotationAligned);
        SmartDashboard.putBoolean("PID ALIGNED", xAligned && yAligned && rotationAligned);
        
        System.out.println(xAligned && yAligned && rotationAligned);
    }

    @Override
    public boolean isFinished() {
        boolean xAligned = xController.atSetpoint();
        boolean yAligned = yController.atSetpoint();
        boolean rotationAligned = rotationController.atSetpoint();

        SmartDashboard.putBoolean("PID ALIGNED", xAligned && yAligned && rotationAligned);

        return (xAligned && yAligned && rotationAligned) || alignedTimer.hasElapsed(PIDAutoAlignConstants.TIMEOUT);
    }

    
}