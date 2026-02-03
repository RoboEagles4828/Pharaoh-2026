package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TunableNumber;

import com.ctre.phoenix6.hardware.CANcoder;


public class ClimbCommand extends Command {

    private final Climber climber;
    private TunableNumber climbStartingPosition = new TunableNumber(ClimberConstants.NT_CLIMB_STARTING_POSITION, ClimberConstants.DEFAULT_CLIMB_STARTING_POSITION); 
    private TunableNumber climbFinalPosition = new TunableNumber(ClimberConstants.NT_CLIMB_FINAL_POSITION, ClimberConstants.DEFAULT_CLIMB_FINAL_POSITION);

    private enum ClimbState{
        RISE,
        RETRACT,
        FINISHED
    }
    private ClimbState state;

    public ClimbCommand(Climber climber) {
        this.climber = climber;
        state = ClimbState.RISE;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setMotorSpeed(0);
        double startingEncoderPosition = climbStartingPosition.get();
    }

    @Override
    public void execute() {
        if (state == ClimbState.RISE) {
            // go up

            if (climber.getPosition() == climbFinalPosition.get()){
                
            }
            // set state to retract
        }
        
        

        if (state == ClimbState.RETRACT) {
            // go down

        
        }

        

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return state == ClimbState.FINISHED;
    }
}