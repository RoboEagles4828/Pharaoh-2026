package frc.robot.subsystems.climber;

public class ClimberConstants {
    public static double DEFAULT_CLIMB_UP_DUTY_CYCLE = -0.2;
    public static double DEFAULT_CLIMB_DOWN_DUTY_CYCLE = 0.2;

    public static final String NT_CLIMB_UP_DUTY_CYCLE = "Tuning/Climber/ClimbUpDutyCycle";
    public static final String NT_CLIMB_DOWN_DUTY_CYCLE = "Tuning/Climber/ClimbDownDutyCycle";

    public static final double CLIMBER_RISE_DELTA = 1;

    public static final String NT_CLIMB_STARTING_POSITION = "Tuning/Climber/StartingPosition";
    public static double DEFAULT_CLIMB_STARTING_POSITION = 0;

    public static final String NT_CLIMB_FINAL_POSITION = "Tuning/Climber/FinalPosition";
    public static double DEFAULT_CLIMB_FINAL_POSITION = 0;
}
