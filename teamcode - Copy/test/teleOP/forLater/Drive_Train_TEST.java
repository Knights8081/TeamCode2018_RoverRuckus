package org.firstinspires.ftc.teamcode.test.teleOP.forLater;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.movement.DrivingMove;

@TeleOp(name = "Drive TEST", group="Nut")

public class Drive_Train_TEST extends OpMode{

    private final HardwareNut robot = new HardwareNut();
    private double[] clawPositions;

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

        double left;
        double right;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        robot.getLeftDrive().setPower(left);
        robot.getRightDrive().setPower(right);
        robot.getLeftArm().setPower(left);
        robot.getRightArm().setPower(right);


    }
}