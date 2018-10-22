package org.firstinspires.ftc.teamcode.test.auto.old;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.constants.RobotConstants;

import java.util.Locale;

/**
 * Created by afield on 11/16/2017.
 */
@Autonomous(name="TEST DRIVING", group="Blue")
@Disabled

public class TESTSENSORENCODER extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareNut robot = new HardwareNut();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    public final static double CLAW_HOME = 0.08;

    final double CLAW_SPEED = 0.02;
    double armPosition = RobotConstants.CLAW_HOME;                   // Servo safe position
    double clawPosition = RobotConstants.CLAW_HOME;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getLeftArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.getLeftDrive().getCurrentPosition(),
                robot.getRightDrive().getCurrentPosition(),
                robot.getRightArm().getCurrentPosition(),
                robot.getRightDrive().getCurrentPosition());


        telemetry.update();


        ColorSensor sensorColor;
        DistanceSensor sensorDistance;


        robot.init(hardwareMap);


        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

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


            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));


                }
            });


            telemetry.update();


            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });


            // Wait for the game to start (driver presses PLAY)



            if (sensorColor.red() >= 60) {
                encoderDrive(DRIVE_SPEED, -5, -5, 3.0);  // S1: Forward 48 Inches with 5 Sec timeout
               sleep(6000);
                encoderDrive(TURN_SPEED, 0, 10, 5.0);
                sleep(6000);
              encoderDrive(DRIVE_SPEED, -10, -10, 5.0);
                sleep(5000);
                stop();
            } else if (sensorColor.blue() >= 60) {
                encoderDrive(DRIVE_SPEED, 1, 1, 3.0);
                robot.getRightClaw().setPosition(CLAW_HOME);
                encoderDrive(DRIVE_SPEED, -3, -3, 4.0);
                encoderDrive(TURN_SPEED, 5, -5, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                encoderDrive(DRIVE_SPEED, -6, -6, 4.0);
                encoderDrive(TURN_SPEED, -8, 8, 4.0);
                encoderDrive(DRIVE_SPEED, -9, -9, 4.0);
                sleep(5000);
                stop();
            }
            if (sensorColor.red() < 60) {
                robot.getLeftDrive().setPower(0);
                robot.getRightDrive().setPower(0);
                robot.getLeftArm().setPower(0);
                robot.getRightArm().setPower(0);
            } else if (sensorColor.blue() < 60) {
                robot.getLeftArm().setPower(0);
                robot.getRightArm().setPower(0);
                robot.getLeftDrive().setPower(0);
                robot.getRightDrive().setPower(0);
            }


            sleep(1000);     // pause for servos to move

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newBackRightTarget;
        int newBackLeftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.getLeftDrive().getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.getRightDrive().getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.getRightArm().getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.getLeftArm().getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);


            robot.getLeftDrive().setTargetPosition(newLeftTarget);
            robot.getRightDrive().setTargetPosition(newRightTarget);
            robot.getRightArm().setTargetPosition(newBackRightTarget);
            robot.getLeftArm().setTargetPosition(newBackLeftTarget);


            // Turn On RUN_TO_POSITION
            robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getRightArm().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getLeftArm().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.getLeftDrive().setPower(Math.abs(speed));
            robot.getRightDrive().setPower(Math.abs(speed));
            robot.getRightArm().setPower(Math.abs(speed));
            robot.getLeftArm().setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.getLeftDrive().isBusy() && robot.getRightDrive().isBusy() && robot.getRightArm().isBusy() && robot.getLeftArm().isBusy()))

            {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.getLeftDrive().getCurrentPosition(),
                        robot.getRightDrive().getCurrentPosition(),
                        robot.getRightArm().getCurrentPosition(),
                        robot.getLeftArm().getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.getLeftDrive().setPower(0);
            robot.getRightDrive().setPower(0);
            robot.getRightArm().setPower(0);
            robot.getLeftArm().setPower(0);

            // Turn off RUN_TO_POSITION
            robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getRightArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getLeftArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move


        }
    }

}
