package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static double DEPLOY_GEAR_RATIO = 20.0; 
    public static double INTAKE_GEAR_RATIO = 4.0; 
    public static double INTAKE_NINJA_STAR_GEAR_RATIO = 5.0;

    public static double WHEEL_DIAMETER = Units.inchesToMeters(2.25);

    public static double DEFAULT_INTAKE_SPEED = 0.5;
    public static final String NT_INTAKE_SPEED_KEY = "Tuning/Intake/IntakeDutyCycle";

    public static double DEFAULT_NINJA_STAR_SPEED = -0.5;
    public static final String NT_NINJA_STAR_SPEED_KEY = "Tuning/Intake/NinjaStarDutyCycle";


    public static final String NT_UPDATE_INTAKE_PID_BUTTON = "Tuning/Intake/UpdatePIDButton";
    public static final String NT_RESET_INTAKE_ENCODER_BUTTON = "Tuning/Intake/ResetDeployEncoder";

    public static final String NT_DEPLOY_P_VALUE = "Tuning/Intake/DeployMotorPValue";
    public static double DEFAULT_DEPLOY_P_VALUE = 1.0;
    public static final String NT_DEPLOY_I_VALUE = "Tuning/Intake/DeployMotorIValue";
    public static double DEFAULT_DEPLOY_I_VALUE = 0.0;
    public static final String NT_DEPLOY_D_VALUE = "Tuning/Intake/DeployMotorDValue";
    public static double DEFAULT_DEPLOY_D_VALUE = 0.0;

    public static final String NT_DEPLOYED_POSITION = "Tuning/Intake/DeployPosition";
    public static double DEFAULT_DEPLOYED_POSITION = 0.5;
    public static final String NT_RAISED_POSITION = "Tuning/Intake/RaisedPosition";
    public static double DEFAULT_RAISED_POSITION = 0.0;

    public static final String NT_INTAKE_S_VALUE = "Tuning/Intake/IntakeMotorSValue";
    public static double DEFAULT_INTAKE_S_VALUE = 0.2;
    public static final String NT_INTAKE_V_VALUE = "Tuning/Intake/IntakeMotorVValue";
    public static double DEFAULT_INTAKE_V_VALUE = 0.35;
    public static final String NT_INTAKE_P_VALUE = "Tuning/Intake/IntakeMotorPValue";
    public static double DEFAULT_INTAKE_P_VALUE = 0.1;
    public static final String NT_INTAKE_I_VALUE = "Tuning/Intake/IntakeMotorIValue";
    public static double DEFAULT_INTAKE_I_VALUE = 0.0;
    public static final String NT_INTAKE_D_VALUE = "Tuning/Intake/IntakeMotorDValue";
    public static double DEFAULT_INTAKE_D_VALUE = 0.0;
}
