package org.firstinspires.ftc.teamcode.test.other.claw;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pd.movement.MoveClaw;

/**
 * This class tests the functionality for a driver opening and closing the claw.
 *
 * If the driver pushes X, the claw will close
 * If the driver pushes B, the claw will open
 *
 * @author Luke Frazer
 */
@TeleOp(name="Nut: Servo test 1", group="Nut")
@Disabled
public class ServoTest extends OpMode {

    private final HardwareNut robot = new HardwareNut();    //reference for robot hardware
    private double[] positions;

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
    }

    @Override
    public void loop() {
        positions = robot.getClawPositions();

        if (gamepad2.x)
            MoveClaw.closeClaw(positions, RobotConstants.CLAW_SPEED);
        else if (gamepad2.b)
            MoveClaw.openClaw(positions, RobotConstants.CLAW_SPEED);

        setPositions();
    }

    private void setPositions() {
        robot.getLeftClaw().setPosition(positions[0]);
        robot.getRightClaw().setPosition(positions[1]);
    }
}
