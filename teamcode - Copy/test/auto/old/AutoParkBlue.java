package org.firstinspires.ftc.teamcode.test.auto.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.constants.RobotConstants;

/**
 * Created by afield on 11/18/2017.
 */
@Autonomous(name="Pushbot: Auto Drive By Encoder 2", group="Pushbot")
@Disabled
public class AutoParkBlue extends LinearOpMode {
    HardwareNut robot = new HardwareNut();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();



    public final static double CLAW_HOME = 0.08;

    final double CLAW_SPEED = 0.02;
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
        // Wait for the game to start (driver presses PLAY)



         // Wait for the game to start (driver presses PLAY)
    waitForStart();

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    encoderDrive(DRIVE_SPEED,  12,  12, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout
    // S3: Reverse 24 Inches with 4 Sec timeout

                sleep(1000);     // pause for servos to move

                telemetry.addData("Path", "Complete");
                telemetry.update();
       stop();

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
