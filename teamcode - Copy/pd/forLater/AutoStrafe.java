package org.firstinspires.ftc.teamcode.pd.forLater;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;

/**
 * This class handles automatic strafing for the robot
 *
 * @author Luke Frazer
 */
public final class AutoStrafe {

    //TODO - Test for value to use for setting power
    private static final float powerValue = 0.9f;

    /**
     * This method handles strafing left
     *
     * @param robot - reference to the actual robot
     */
    public static void left(final HardwareNut robot) {

        //Sequence: rD -, lD +, rA +, lA -
        robot.getRightDrive().setPower(-powerValue);
        robot.getLeftDrive().setPower(powerValue);
        robot.getRightArm().setPower(powerValue);
        robot.getLeftArm().setPower(-powerValue);
    }

    /**
     * This method handles strafing right
     *
     * @param robot - reference to the actual robot
     */
    public static void right(final HardwareNut robot) {

        //Sequence: rD +, lD -, rA -, lA +
        robot.getRightDrive().setPower(powerValue);
        robot.getLeftDrive().setPower(-powerValue);
        robot.getRightArm().setPower(-powerValue);
        robot.getLeftArm().setPower(powerValue);
    }
}
