package org.firstinspires.ftc.teamcode.pd.movement;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pd.constants.RobotConstants;

/**
 * This class contains functions to update the positions for the hands on the claw
 *
 * @author Corbin Young
 */
public final class MoveHand {

    /**
     * This function moves the arms of the claw outward so that the claw "opens" and lets go
     *  of whatever object it may or may not have been holding onto
     *
     *  positions[0] -> left claw position
     *  positions[1] -> right claw position
     *
     * @param handpositions - array holding the positions for the hands for the claw
     * @param speedhand - speed at which the claw will change positions
     */
    public static void openHand(final double[] handpositions, final double speedhand) {
        /* Update positions */
        handpositions[0] += speedhand;
        handpositions[1] -= speedhand;

        /* Clip the positions to make sure they are within the valid range for the servos */
        handpositions[0]  = Range.clip(handpositions[0], RobotConstants.IDOLHAND_MIN_RANGE, RobotConstants.IDOLHAND_MAX_RANGE);
        handpositions[1] = Range.clip(handpositions[1], RobotConstants.IDOLHAND_MIN_RANGE, RobotConstants.IDOLHAND_MAX_RANGE);
    }

    /**
     * This function moves the arms of the claw inward so that the claw "closes" and grabs
     *  whatever object may or may not be in front of it
     *
     *  positions[0] -> left claw position
     *  positions[1] -> right claw position
     *
     * @param handpositions - array holding the positions for the hands for the claw
     * @param speedhand - speed at which the claw will change positions
     */
    public static void closeHand(final double[] handpositions, final double speedhand) {
        /* Update positions */
        handpositions[0] -= speedhand;
        handpositions[1] += speedhand;

        /* Clip the positions to make sure they are within the valid range for the servos */
        handpositions[0]  = Range.clip(handpositions[0], RobotConstants.IDOLHAND_MIN_RANGE, RobotConstants.IDOLHAND_MAX_RANGE);
        handpositions[1] = Range.clip(handpositions[1], RobotConstants.IDOLHAND_MIN_RANGE, RobotConstants.IDOLHAND_MAX_RANGE);
    }
}
