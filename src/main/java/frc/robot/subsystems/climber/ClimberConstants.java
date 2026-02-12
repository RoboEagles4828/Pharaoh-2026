package frc.robot.subsystems.climber;

public class ClimberConstants {

    public static final double GEAR_RATIO = 20.0; 

    public static final String NT_CLIMB_UP_DUTY_CYCLE = "Tuning/Climber/ClimbUpDutyCycle";
    public static double DEFAULT_CLIMB_UP_DUTY_CYCLE = -0.2;
    public static final String NT_CLIMB_DOWN_DUTY_CYCLE = "Tuning/Climber/ClimbDownDutyCycle";
    public static double DEFAULT_CLIMB_DOWN_DUTY_CYCLE = 0.2;

    // we seed the starting position to this, so 0.0 is 'fully down' in match cases.
    public static double CLIMBER_START_POSITION = 0.0;

    public static final String NT_CLIMB_PEAK_POSITION = "Tuning/Climber/PeakPosition";
    public static double DEFAULT_CLIMB_PEAK_POSITION = -1.5;
    public static final String NT_CLIMB_FINAL_POSITION = "Tuning/Climber/FinalPosition";
    public static double DEFAULT_CLIMB_FINAL_POSITION = 0;

    public static final String NT_CLIMBER_P_VALUE = "Tuning/Climber/ClimberPValue";
    public static double DEFAULT_CLIMBER_P_VALUE = 3.2;
    public static final String NT_CLIMBER_I_VALUE = "Tuning/Climber/ClimberIValue";
    public static double DEFAULT_CLIMBER_I_VALUE = 0.0;
    public static final String NT_CLIMBER_D_VALUE = "Tuning/Climber/ClimberDValue";
    public static double DEFAULT_CLIMBER_D_VALUE = 0.05;

    public static final String NT_CLIMBER_ZERO_ENCODER_BTN = "Tuning/Climber/ZeroEncoderBtn";
}
