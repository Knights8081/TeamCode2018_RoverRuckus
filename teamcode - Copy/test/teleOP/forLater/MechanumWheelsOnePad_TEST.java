package org.firstinspires.ftc.teamcode.test.teleOP.forLater;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.Position;

//import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pd.movement.DrivingMove;



/**
 * Created 10/18/2017
 *
 * This Opmode is used to control the robot during the teleOp period of the competition
 *
 * @author Anna Field
 */
@TeleOp(name="Nut: MechanumWheelsOnePad_TEST", group="Nut")

public class MechanumWheelsOnePad_TEST extends OpMode {

    private final HardwareNut robot = new HardwareNut();
    private ElapsedTime runtime = new ElapsedTime();
//    reference for robot hardware
//    private double[] clawPositions;                             //handles updating positions for the claw
//    private double[] handPositions;

    /* Game pad controller reference declarations */


    /**
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");


     //   robot.getRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
//        Move.runMovementTest(robot);  //This is a test function that only is run for debugging issues involving movement
    }

    @Override
    public void loop() {

        /* CHECK FOR CLAW UPDATE ----------------------------------------------------------------*/
//        clawPositions = robot.getClawPositions();
//        handPositions = robot.getIdolHandPosition();

        /* CONTROL THE CLAW --------------------------------------------------------------------*/
//        if (gamepad1.b)
//            MoveClaw.closeClaw(clawPositions, RobotConstants.CLAW_SPEED);
//        else if (gamepad1.x)
//            MoveClaw.openClaw(clawPositions, RobotConstants.CLAW_SPEED);

//        if(gamepad1.a) {
//            robot.getRightDrive().setPower(Range.clip(robot.getRightDrive().getPower() + 0.01, -1.0, 1.0));
//            robot.getLeftDrive().setPower(Range.clip(robot.getLeftDrive().getPower() + 0.01, -1.0, 1.0));
//            robot.getLeftArm().setPower(Range.clip(robot.getLeftArm().getPower() + 0.01, -1.0, 1.0));
//            robot.getRightArm().setPower(Range.clip(robot.getRightArm().getPower() + 0.01, -1.0, 1.0));
//        }
//        else {
//            robot.getRightDrive().setPower(0);
//        }


//        /* CONTROL THE IDOL --------------------------------------------------------------------*/
//        if (gamepad1.right_bumper)
//            robot.getIdolHand().setPosition(1.0);
//        else if (gamepad1.left_bumper)
//            robot.getIdolHand().setPosition(0.0);


//        if (gamepad1.right_bumper)
//            robot.getIdolHand().setPosition(.9);
//        else if (gamepad1.left_bumper)
//            robot.getIdolHand().setPosition(.2);

//        /* DECLARE CLAW POSITIONS --------------------------------------------------------------*/
//        robot.gettopLeftClaw().setPosition(clawPositions[0]);
//        robot.getLeftClaw().setPosition(clawPositions[0]);
//        robot.getRightClaw().setPosition(clawPositions[1]);
//        robot.gettopRightClaw().setPosition(clawPositions[1]);

//        /* CHECK FOR IDOL SLIDE UPDATE ----------------------------------------------------------*/
//        if (gamepad1.dpad_up)
//            robot.getLiftArm().setPower(-.90);
//        else if (gamepad1.dpad_down)
//            robot.getLiftArm().setPower(.55);
//        else
//            robot.getLiftArm().setPower(0.0);
//    }
//}

        /* DRIVE ROBOT --------------------------------------------------------------------------*/
        double drive = -gamepad1.left_stick_y;   // Power for forward and back motion; Negative because the gamepad is weird
        double strafe = gamepad1.left_stick_x;  // Power for left and right motion
        double rotate = gamepad1.right_stick_x;  // Power for rotating the robot

        DrivingMove.drive(robot, drive, strafe, rotate);


//        /*Test run to position in teleop*/


//        int Position = 0;
//
//        if (gamepad1.a) {
//            Position = 1;
//        }
//
//        if (gamepad1.b) {
//            Position = 2;
//        }
//
//            switch (Position) {
//
//                case 0:
//                    encoderDrive(0, 0, 0, 0);
//
//                case 1:
//                    encoderDrive(RobotConstants.DRIVE_SPEED, 100, 100, 1);
//                    break;
//
//                case 2:
//                    encoderDrive(RobotConstants.DRIVE_SPEED, -100, -100, 1);
//
//            }
//
//    }
//
//    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
//
//        int newLeftDriveTarget = robot.getLeftDrive().getCurrentPosition() + (int)(leftInches * RobotConstants.COUNTS_PER_INCH);
//        int newRightDriveTarget = robot.getRightDrive().getCurrentPosition() + (int)(rightInches * RobotConstants.COUNTS_PER_INCH);
//
//        robot.getLeftDrive().setTargetPosition(newLeftDriveTarget);
//        robot.getRightDrive().setTargetPosition(newRightDriveTarget);
//
//        turnOnRunToPosition();
//
//        runtime.reset();
//        setAllPower(Math.abs(speed));
//
//        setAllPower(0.0);
//
//        turnOnRunUsingEncoder();
//    }
//
//    private boolean allMotorsBusy() {
//        return robot.getLeftDrive().isBusy() && robot.getRightDrive().isBusy();
//    }
//
//    /**
//     * Check if any motor is busy
//     *
//     * @return true if any motor is busy, false if all are not busy
//     */
//    private boolean anyMotorBusy() {
//        return robot.getLeftDrive().isBusy() || robot.getRightDrive().isBusy();
//
//    }
//
//    /**
//     * Set all motors to mode: RUN_TO_POSITION
//     */
//    private void turnOnRunToPosition() {
//        robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    /**
//     * Set all motors to mode: RUN_USING_ENCODER
//     */
//    private void turnOnRunUsingEncoder() {
//        robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    /**
//     * Set all motors to mode: STOP_AND_RESET_ENCODER
//     */
//    private void turnOnStopAndReset() {
//        robot.getLeftDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.getRightDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    /**
//     * Set the power of all motors to specified value
//     *
//     * @param power - value for power
//     */
//    private void setAllPower(final double power) {
//        robot.getLeftDrive().setPower(power);
//        robot.getRightDrive().setPower(power);
//    }
    }
}
