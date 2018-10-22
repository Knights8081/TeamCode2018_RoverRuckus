package org.firstinspires.ftc.teamcode.test.other.arms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;

/**
 * This class will perform a brief test of moving the lift arm up and down
 *
 * @author Corbin Young
 */
@TeleOp(name="Nut: Lift Arm Test 1", group="Nut")
@Disabled
public class LiftArmTest extends OpMode {

    private final HardwareNut robot = new HardwareNut();

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.getLiftArm().setPower(gamepad2.left_stick_y);
    }

}
