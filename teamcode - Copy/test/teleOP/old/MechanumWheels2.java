package org.firstinspires.ftc.teamcode.test.teleOP.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.old.StrafeByHand;

/**
 * Created by afield on 10/18/2017.
 */
@TeleOp(name="Nut: MechanumWheels2", group="Nut")
@Disabled
public class MechanumWheels2 extends OpMode {
    /* Declare OpMode members. */
    HardwareNut robot = new HardwareNut(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double clawOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.02;             // sets rate to move servo
    double RT = 0.0;
    double LT = 0.0;
    double lift_up = 0.0;


    /*
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

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    }

    @Override
    public void loop() {
//        double left;
//        double right;
//        double RT;
//        double LT;


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        double RT = gamepad1.right_trigger;
        double LT = gamepad1.left_trigger;



        robot.getLeftDrive().setPower(left);
        robot.getRightDrive().setPower(right);
        robot.getLeftArm().setPower(left);
        robot.getRightArm().setPower(right);



        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        if (gamepad2.dpad_up)
            robot.getLiftArm().setPower(.75);
        else if (gamepad2.dpad_down)
            robot.getLiftArm().setPower(-.75);

        robot.getLiftArm().setPower(0.0);



        if (RT > 0.1)
            StrafeByHand.right(robot, RT);
        else if (LT > 0.1)
            StrafeByHand.left(robot, LT);


    }

}

