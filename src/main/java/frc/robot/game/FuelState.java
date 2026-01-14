package frc.robot.game;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Defines the various states a fuel game piece can enter while the robot interacts with it.
 */
public enum FuelState {
    /** There is no fuel in the robot (that we know of). */
    EMPTY,
    /** The robot will actively intake fuel from the ground. */
    INTAKE,
    /** The robot has a properly loaded fuel and will carry it. */
    CARRY,
    /** The robot is preparing to score (shooter wheels getting up to speed). */
    PREPARE_TO_SCORE,
    /** The robot is ready to score its fuel. */
    READY_TO_SCORE,
    /** The robot will actively score the fuel into the hub. */
    SCORE,
    /** The fuel is jammed in the hopper. Agitation will be performed. */
    HOPPER_JAMMED;

    /** The current fuel state. We always start a match with 8 fuel. */
    private static FuelState currentState = CARRY;
    /** A trigger for each enumeration value. */
    private final Trigger trigger;

    /**
     * Called to create each enumeration value. A trigger for each value.
     */
    private FuelState() {
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
    public static FuelState getCurrentState() {
        return currentState;
    }

    /**
     * Sets the next fuel state.
     * 
     * @param nextState
     *            the state to set. If null, the state does not change.
     */
    public static void setCurrentState(final FuelState nextState) {
        if (nextState != null) {
            currentState = nextState;
        }
    }
}