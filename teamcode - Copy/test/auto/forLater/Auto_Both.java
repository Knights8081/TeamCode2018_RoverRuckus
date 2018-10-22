package org.firstinspires.ftc.teamcode.test.auto.forLater;

/**
 * Created by afield on 3/5/2018.
 */
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.test.auto.forLater.SensorREVColorDistance;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="AUTO PHAT TEST RED BOTH", group="PushBot")
@Disabled
public class Auto_Both extends LinearOpMode {

    /* Declare OpMode members. */

    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_MOTOR_REV    = 288 ;  // eg: TETRIX Motor Encoder
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double     DRIVE_SPEED             = 0.6;
    private static final double     TURN_SPEED              = 0.5;

    VuforiaLocalizer vuforia;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    OpenGLMatrix lastLocation = null;

    HardwareNut robot = new HardwareNut();

    SensorREVColorDistance hwSensor = new SensorREVColorDistance();
    private ElapsedTime runtime = new ElapsedTime();


    /**
     * TODO - What is this method doing?
     */
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        hwSensor.init();

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        sensorColor.enableLed(true);


        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         *
         */


            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();

            //Reset encoders
            turnOnStopAndReset();

            //Turn on RUN_USING_ENCODER
            turnOnRunUsingEncoder();

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0", "Starting at %7d :%7d",
                    robot.getLeftDrive().getCurrentPosition(),
                    robot.getRightDrive().getCurrentPosition());
            telemetry.update();

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = "AXe+G0X/////AAAAGWX2PnVHaUK0lbybIB3Luad39Qzmm36RMGPUaiMP4ybPeYkYi7gHFkzHdn5vRGY+J7+0ZFIAVXqKf4PgJP8TpGTqnw8HM/aCP3BpmmYd5nmJxfwxQxzwydbjTovUJGcGqm+FBJVI93jCnHT68WPIv0Aki5yLpXXU6FwlYuzNMixa7FOTIdew60kBpM70uhIX8Y5yx+GKTB2WL3Sl0UKPE7jwWep0BsQe0gidcJdMXQAVU13jBrpXBKUSmKIuuQhWFUax9d/yaOns9mheTCH9S4RgQ96Ln4LwB0SzpIB2/rgQHkSgm7MWmCzsDn95/hRMYhSH82CevjgHqvTvRNIqwZ+caom47CVfgxzmi2iJNu+d";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            relicTrackables.activate();


            // Step through each leg of the path,
            // NOTE: Reverse movement is obtained by setting a negative distance (not speed)


//        robot.getLeftClaw().setPosition(HardwareNut.CLAW_MIN_RANGE);            // S4: Stop and close the claw.
//        robot.getRightClaw().setPosition(HardwareNut.CLAW_MAX_RANGE);
//
            sleep(1500);
//        robot.getLiftArm().setPower(.25);
//        sleep(700);
//        robot.getLiftArm().setPower(0.0);
//        sleep(300);

            while (opModeIsActive()) {

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();


                if (vuMark == RelicRecoveryVuMark.RIGHT) {


                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
//                robot.getLeftClaw().setPosition(.9);            // S4: Stop and close the claw.
                    armDown(2.0);
                    jewel(1.0);
                    robot.getRightClaw().setPosition(-.9);
                    sleep(1500);
                    encoderDrive(DRIVE_SPEED, -28, -28, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout
                    robot.getClawLift().setPower(.5);
                    sleep(300);
                    robot.getClawLift().setPower(0.0);
                    encoderDrive(TURN_SPEED, -19, 19, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                    encoderDrive(DRIVE_SPEED, -12, -12, 4.0);  // S3: Reverse 12 Inches with 4 Sec timeout
                    sleep(1500);
//                robot.getLeftClaw().setPosition(.4);
                    robot.getRightClaw().setPosition(.7);
                    sleep(1500);
                    encoderDrive(DRIVE_SPEED, 5, 5, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout

                    stop();
                } else if (vuMark == RelicRecoveryVuMark.LEFT) {


                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
//                robot.getLeftClaw().setPosition(.9);            // S4: Stop and close the claw.
                    armDown(2.0);
                    jewel(1.0);
                    robot.getRightClaw().setPosition(-.9);
                    sleep(1500);
                    encoderDrive(DRIVE_SPEED, -43, -43, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout
                    robot.getClawLift().setPower(.5);
                    sleep(300);
                    robot.getClawLift().setPower(0.0);
                    encoderDrive(TURN_SPEED, -19, 19, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                    encoderDrive(DRIVE_SPEED, -12, -12, 4.0);  // S3: Reverse 12 Inches with 4 Sec timeout
                    sleep(1500);
//                robot.getLeftClaw().setPosition(.4);
                    robot.getRightClaw().setPosition(.7);
                    sleep(1500);
                    encoderDrive(DRIVE_SPEED, 5, 5, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout

                    stop();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {


                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
//                robot.getLeftClaw().setPosition(.9);            // S4: Stop and close the claw.
                    armDown(2.0);
                    jewel(1.0);
                    robot.getRightClaw().setPosition(-.9);
                    sleep(1500);
                    encoderDrive(DRIVE_SPEED, -35.5, -35.5, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout
                    robot.getClawLift().setPower(.5);
                    sleep(300);
                    robot.getClawLift().setPower(0.0);
                    encoderDrive(TURN_SPEED, -19, 19, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                    encoderDrive(DRIVE_SPEED, -12, -12, 4.0);  // S3: Reverse 12 Inches with 4 Sec timeout
                    sleep(1500);
//                robot.getLeftClaw().setPosition(.4);
                    robot.getRightClaw().setPosition(.7);
                    sleep(1500);
                    encoderDrive(DRIVE_SPEED, 5, 5, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout

                    stop();
                }


            }


//        robot.getLeftClaw().setPosition(HardwareNut.CLAW_MAX_RANGE);            // S4: Stop and close the claw.
//        robot.getRightClaw().setPosition(HardwareNut.CLAW_MIN_RANGE);
            //      sleep(1500);     // pause for servos to move

//        telemetry.addData("Path", "Complete");
//        telemetry.update();
            telemetry.update();
            sleep(500);

        }
    }

    /**
     * Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     * @param speed
     * @param leftInches
     * @param rightInches
     * @param timeoutS
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int newLeftDriveTarget = robot.getLeftDrive().getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            int newRightDriveTarget = robot.getRightDrive().getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            int newLeftArmTarget = robot.getLeftArm().getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            int newRightArmTarget = robot.getRightArm().getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            robot.getLeftDrive().setTargetPosition(newLeftDriveTarget);
            robot.getRightDrive().setTargetPosition(newRightDriveTarget);
            robot.getLeftArm().setTargetPosition(newLeftArmTarget);
            robot.getRightArm().setTargetPosition(newRightArmTarget);

            // Turn On RUN_TO_POSITION
            turnOnRunToPosition();

            // reset the timeout time and start motion.
            runtime.reset();
            setAllPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && allMotorsBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftDriveTarget,  newRightDriveTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.getLeftDrive().getCurrentPosition(),
                        robot.getRightDrive().getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            setAllPower(0.0);

            // Turn off RUN_TO_POSITION
            turnOnRunUsingEncoder();

            //  sleep(250);   // optional pause after each move
        }
    }

    /**
     * Check if all motors are busy
     *
     * @return true if all motors are busy, false if any motor is not busy
     */
    private boolean allMotorsBusy() {
        return robot.getLeftDrive().isBusy() && robot.getRightDrive().isBusy() && robot.getLeftArm().isBusy() && robot.getRightArm().isBusy();
    }

    /**
     * Check if any motor is busy
     *
     * @return true if any motor is busy, false if all are not busy
     */
    private boolean anyMotorBusy() {
        return robot.getLeftDrive().isBusy() || robot.getRightDrive().isBusy() || robot.getLeftArm().isBusy() || robot.getRightArm().isBusy();
    }

    /**
     * Set all motors to mode: RUN_TO_POSITION
     */
    private void turnOnRunToPosition() {
        robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getLeftArm().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightArm().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Set all motors to mode: RUN_USING_ENCODER
     */
    private void turnOnRunUsingEncoder() {
        robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getLeftArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set all motors to mode: STOP_AND_RESET_ENCODER
     */
    private void turnOnStopAndReset() {
        robot.getLeftDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLeftArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Set the power of all motors to specified value
     *
     * @param power - value for power
     */


    private void setAllPower(final double power) {
        robot.getLeftDrive().setPower(power);
        robot.getRightDrive().setPower(power);
        robot.getLeftArm().setPower(power);
        robot.getRightArm().setPower(power);
    }

    public void jewel ( double holdTime){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {




            if (sensorColor.blue() > 30) {

                sleep(1000);
                encoderDrive(TURN_SPEED, -3, 3, 2);
                robot.getBallarm().setPower(-.8);
                sleep(2300);
                robot.getBallarm().setPower(0.0);
                encoderDrive(DRIVE_SPEED, 3.5, -3.5, 2);
                sleep(1000);
//                robot.getBallarm().setPosition(0.0);
            }
            else if (sensorColor.red() > 30) {

                sleep(1000);
                encoderDrive(TURN_SPEED, 3, -3, 2);
                robot.getBallarm().setPower(-.8);
                sleep(2300);
                robot.getBallarm().setPower(0.0);
                encoderDrive(TURN_SPEED, -3.5, 3.5, 2);
                sleep(1000);
                // robot.getBallarm().setPosition(0.0);
            }
            else {
                sleep(3000);
                stop();

            }
        }

    }

    public void armDown(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime)
            robot.getBallarm().setPower(.5);
        sleep(1400);
        robot.getBallarm().setPower(0.0);
    }

}
