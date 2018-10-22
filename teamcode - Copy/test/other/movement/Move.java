package org.firstinspires.ftc.teamcode.test.other.movement;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;

/**
 * This class will test the movement of the robot. It will test 6 key movement features:
 *
 *   1) Moving forward
 *   2) Moving backward
 *   3) Turning left
 *   4) Turning right
 *   5) Strafing Left
 *   6) Strafing right
 *
 * @author Corbin Young
 */
public final class Move {

    /* SPEED VALUES ------------------------------------------------*/
    private static final float POWERVALUE = 0.75f;
    private static final float STRAFEVALUE = 0.6f;
    private static final float STOPVALUE = 0.0f;

    /* TIME VALUES -------------------------------------------------*/
    private static final long SHORTWAIT = 250;      //250ms pause
    private static final long LONGWAIT = 425;       //425ms pause

    private static HardwareNut robot;       //instance of the robot to test

    /**
     * Move the robot forward
     */
    private static void forward() {
        robot.getRightDrive().setPower(-POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(-POWERVALUE);
    }

    /**
     * Move the robot backward
     */
    private static void backward() {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(POWERVALUE);
        robot.getRightArm().setPower(POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Turn the robot to the left
     */
    private static void turnLeft() {
        robot.getRightDrive().setPower(-POWERVALUE);
        robot.getLeftDrive().setPower(POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Turn the robot to the right
     */
    private static void turnRight() {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(POWERVALUE);
        robot.getLeftArm().setPower(-POWERVALUE);
    }

    /**
     * Strafe the robot to the left
     */
    private static void strafeLeft() {
        robot.getRightDrive().setPower(-STRAFEVALUE);
        robot.getLeftDrive().setPower(STRAFEVALUE);
        robot.getRightArm().setPower(STRAFEVALUE);
        robot.getLeftArm().setPower(-STRAFEVALUE);
    }

    /**
     * Strafe the robot to the right
     */
    private static void strafeRight() {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Stops the robot so that it is no longer moving or turning
     */
    private static void stop() {
        robot.getRightDrive().setPower(STOPVALUE);
        robot.getLeftDrive().setPower(STOPVALUE);
        robot.getRightArm().setPower(STOPVALUE);
        robot.getLeftArm().setPower(STOPVALUE);
    }

    /**
     * Test function to run through all of the movement tests
     */
    public static void runMovementTest(final HardwareNut r) {
        robot = r;

        forward();
        SystemClock.sleep(SHORTWAIT);

        stop();
        SystemClock.sleep(LONGWAIT);

        backward();
        SystemClock.sleep(SHORTWAIT);

        stop();
        SystemClock.sleep(LONGWAIT);

        turnLeft();
        SystemClock.sleep(SHORTWAIT);

        stop();
        SystemClock.sleep(LONGWAIT);

        turnRight();
        SystemClock.sleep(SHORTWAIT);

        stop();
        SystemClock.sleep(LONGWAIT);

        strafeLeft();
        SystemClock.sleep(SHORTWAIT);

        stop();
        SystemClock.sleep(LONGWAIT);

        strafeRight();
        SystemClock.sleep(SHORTWAIT);

        stop();
        SystemClock.sleep(LONGWAIT);
    }
}
