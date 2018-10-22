package org.firstinspires.ftc.teamcode.test.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pd.movement.DrivingMove;
import org.firstinspires.ftc.teamcode.pd.movement.MoveClaw;

/**
 * Created 10/18/2017
 *
 * This Opmode is used to control the robot during the teleOp period of the competition
 *
 * @author Anna Field
 */
@TeleOp(name="Nut: MechanumWheelsOnePad_NEW", group="Nut")
public class MechanumWheelsOnePad_NEW extends OpMode {

    private final HardwareNut robot = new HardwareNut();        //reference for robot hardware
    private double[] clawPositions;                             //handles updating positions for the claw
    private double[] handPositions;

    /* Game pad controller reference declarations */
    private double left;
    private double right;
    private double left2;
    private double RT;
    private double LT;

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
        telemetry.addData("Say", "Hello Driver");    //
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
        /* SET REFERENCES -----------------------------------------------------------------------*/
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        RT = gamepad1.right_trigger;
        LT = gamepad1.left_trigger;

        /* CHECK FOR CLAW UPDATE ----------------------------------------------------------------*/

        clawPositions = robot.getClawPositions();
        handPositions = robot.getIdolHandPosition();


        /* CONTROL THE CLAW --------------------------------------------------------------------*/
        if (gamepad1.b)
            MoveClaw.closeClaw(clawPositions, RobotConstants.CLAW_SPEED);
        else if (gamepad1.x)
            MoveClaw.openClaw(clawPositions, RobotConstants.CLAW_SPEED);


/* CONTROL THE IDOL --------------------------------------------------------------------*/

        if (gamepad1.right_bumper)
            robot.getIdolHand().setPosition(1.0);
        else if (gamepad1.left_bumper)
            robot.getIdolHand().setPosition(0.0);

        if (gamepad1.dpad_right)
            robot.getIdolWrist().setPosition(1.0);
        else if (gamepad1.dpad_left)
            robot.getIdolWrist().setPosition(0.0);

        if (gamepad1.y)
            robot.getClawLift().setPower(-.6);
        else if (gamepad1.a)
            robot.getClawLift().setPower(.6);
        else
            robot.getClawLift().setPower(0);

        if (gamepad1.right_trigger > .1)
            robot.getIdolSlide().setPower(6);
        else if (gamepad1.left_trigger > .1)
            robot.getIdolSlide().setPower(-6);
        else
            robot.getIdolSlide().setPower(0);


/* DECLARE CLAW POSITIONS --------------------------------------------------------------*/

        robot.gettopLeftClaw().setPosition(clawPositions[0]);
        robot.getLeftClaw().setPosition(clawPositions[0]);
        robot.getRightClaw().setPosition(clawPositions[1]);
        robot.gettopRightClaw().setPosition(clawPositions[1]);


/* SET ARM POWER ------------------------------------------------------------------------*/

       robot.getLiftArm().setPower(.1*-left2);


/* CHECK FOR IDOL SLIDE UPDATE ----------------------------------------------------------*/

        if (gamepad1.dpad_up)
            robot.getLiftArm().setPower(-.90);
        else if (gamepad1.dpad_down)
            robot.getLiftArm().setPower(.80);
        else
            robot.getLiftArm().setPower(0.0);


        /* DRIVE ROBOT --------------------------------------------------------------------------*/
        double drive = -gamepad1.left_stick_y;   // Power for forward and back motion; Negative because the gamepad is weird
        double strafe = gamepad1.left_stick_x;  // Power for left and right motion
        double rotate = gamepad1.right_stick_x;  // Power for rotating the robot

        DrivingMove.drive(robot, drive, strafe, rotate);
    }
}
