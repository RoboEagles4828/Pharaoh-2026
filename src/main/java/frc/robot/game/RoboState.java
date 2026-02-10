package frc.robot.game;

import java.util.EnumMap;
import java.util.EnumSet;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Defines the various states a fuel game piece can enter while the robot interacts with it.
 */
public enum RoboState {
    /** The robot will actively intake fuel from the ground. */
    INTAKE,
    /** The robot has a properly loaded fuel and will carry it. */
    CARRY,
    /** The robot is preparing to score (shooter wheels getting up to speed). */
    PREPARE_TO_SCORE,
    /** The robot will actively score the fuel into the hub. */
    SCORE,
    /** The robot will prepare to climb by getting into the correct position and setting up the climber */
    PREPARE_TO_CLIMB,
    /** The robot will start climbing the tower. */
    CLIMB,
    /** The robot will prepare to pass (shooter getting into right configuration) */
    PREPARE_TO_PASS,
    /** The robot will pass the fuel to the target location*/
    PASS;

    private static final EnumMap<RoboState, EnumSet<RoboState>> ALLOWED_TRANSITIONS = new EnumMap<>(RoboState.class);

    static {
        ALLOWED_TRANSITIONS.put(CARRY, EnumSet.of(
            INTAKE,
            PREPARE_TO_SCORE,
            PREPARE_TO_PASS,
            PREPARE_TO_CLIMB
        ));

        ALLOWED_TRANSITIONS.put(INTAKE, EnumSet.of(
            CARRY
        ));

        ALLOWED_TRANSITIONS.put(PREPARE_TO_SCORE, EnumSet.of(
            SCORE,
            CARRY
        ));

        ALLOWED_TRANSITIONS.put(SCORE, EnumSet.of(
            CARRY
        ));

        ALLOWED_TRANSITIONS.put(PREPARE_TO_PASS, EnumSet.of(
            PASS,
            CARRY
        ));

        ALLOWED_TRANSITIONS.put(PASS, EnumSet.of(
            CARRY
        ));

        ALLOWED_TRANSITIONS.put(PREPARE_TO_CLIMB, EnumSet.of(
            CLIMB,
            CARRY
        ));

        ALLOWED_TRANSITIONS.put(CLIMB, EnumSet.noneOf(RoboState.class));
    }


    /** The current fuel state. We always start a match with 8 fuel. */
    private static RoboState currentState = CARRY;

    /** A trigger for each enumeration value. */
    private final Trigger trigger;

    /**
     * Called to create each enumeration value. A trigger for each value.
     */
    private RoboState() {
        this.trigger = new Trigger(this::isCurrent);
    }

    /**
     * @return true if this enum value is the current state, or false otherwise.
     */
    public boolean isCurrent() {
        return getCurrentState() == this;
    }

    /**
     * @return the trigger for this enumeration value.
     */
    public Trigger getTrigger() {
        return this.trigger;
    }

    /**
     * @return the current fuel state (never null).
     */
    public static RoboState getCurrentState() {
        return currentState;
    }

    /**
     * Sets the next fuel state.
     * 
     * @param nextState
     *            the state to set. If null, the state does not change.
     */
    public static void setCurrentState(final RoboState nextState) {
        if (nextState != null) {
            currentState = nextState;
        }
    }

    public static boolean transitionStateTo(RoboState next) {
        if (next == null)
            return false;

        RoboState current = currentState;

        EnumSet<RoboState> allowed = ALLOWED_TRANSITIONS.get(current);
        if (allowed == null || !allowed.contains(next)) {
            DriverStation.reportWarning("Blocked invalid state transition: " + current + " → " + next, false);
            return false;
        }

        currentState = next;
        return true;
    }
}