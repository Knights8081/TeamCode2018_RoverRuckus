package org.firstinspires.ftc.teamcode.pd.forLater;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pd.constants.RobotConstants;

/**
 * Created by afield on 4/5/2018.
 */

public class HardwareNEW {

    /* Define Motor references ------------------------------------------------------------------*/
    private DcMotor  frontLeft   = null;        // Front left wheel
    private DcMotor  frontRight  = null;        // Front right wheel
    private DcMotor  backLeft     = null;        // Back left wheel
    private DcMotor  backRight    = null;        // Back right wheel
    private DcMotor  liftArm     = null;
    private DcMotor  idolLift    = null;
    private DcMotor  barm        = null;
    private DcMotor  sensorArm    = null;

    /* Define Servo references ------------------------------------------------------------------*/
    private Servo    leftClaw    = null;
    private Servo    rightClaw   = null;
    private Servo    topLeftClaw   = null;
    private Servo    topRightClaw   = null;
    private Servo    idolHand    = null;
    private Servo    idolDead   = null;

    /**
     * Creates an instance of the Hardware Nut class
     */
    public HardwareNEW(){
    }

    /* Motor getters ----------------------------------------------------------------------------*/
    public DcMotor getfrontLeft() {
        return frontLeft;
    }

    public DcMotor getfrontRight() {
        return frontRight;
    }

    public DcMotor getbackLeft() {
        return backLeft;
    }

    public DcMotor getbackRight() {
        return backRight;
    }

    public DcMotor getLiftArm() {
        return liftArm;
    }

//    }

    public DcMotor getIdolLift() {
        return idolLift;
    }

    public DcMotor getBarm() { return barm; }

    public DcMotor getsensorArm() {
        return sensorArm;
    }
    /* Servo getters ----------------------------------------------------------------------------*/
    public Servo getLeftClaw() {
        return leftClaw;
    }

    public Servo gettopLeftClaw() {
        return topLeftClaw;
    }

    public Servo gettopRightClaw() {
        return topRightClaw;
    }

    public Servo getRightClaw() {
        return rightClaw;
    }

    public Servo getIdolHand() {
        return idolHand;
    }

    public Servo getIdolDead() { return idolDead;}

    public double[] getClawPositions() {
        return new double[]{leftClaw.getPosition(), rightClaw.getPosition(), topLeftClaw.getPosition(), topRightClaw.getPosition()};
    }

    public double[] getIdolHandPosition() {

        return new double[]{idolHand.getPosition(), idolDead.getPosition()};
    }

    /* Functional methods -----------------------------------------------------------------------*/

    /**
     * Initialize all standard hardware interfaces
     *
     * @param hwMap - reference to the hardware map on the user interface program
     */
    public void init(final HardwareMap hwMap) {

        /* INITIALIZE MOTORS --------------------------------------------------------------------*/

        /* Wheel Motors */
        frontLeft  = hwMap.get(DcMotor.class, "front_left");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backLeft    = hwMap.get(DcMotor.class, "back_left");
        backRight   = hwMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);  // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        /* Arm Motors */
        liftArm = hwMap.get(DcMotor.class, "lift_arm");
        idolLift = hwMap.get(DcMotor.class, "idol_lift");
        sensorArm = hwMap.get(DcMotor.class, "sensor_arm");


        // Set all motors to zero power
        /* SET INITIAL POWER --------------------------------------------------------------------*/
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        liftArm.setPower(0);
        sensorArm.setPower(0);
        idolLift.setPower(0);

        /* SET MOTOR MODE -----------------------------------------------------------------------*/
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idolLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sensorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* INITIALIZE SERVOS --------------------------------------------------------------------*/
        topRightClaw  = hwMap.get(Servo.class, "top_right_claw");
        rightClaw  = hwMap.get(Servo.class, "right_claw");
        leftClaw = hwMap.get(Servo.class, "left_claw");
        topLeftClaw  = hwMap.get(Servo.class, "top_left_claw");
        idolHand = hwMap.get(Servo.class, "idol_hand");
        idolDead = hwMap.get(Servo.class, "idol_dead");

        rightClaw.setPosition(RobotConstants.CLAW_MIN_RANGE);
        leftClaw.setPosition(RobotConstants.CLAW_MAX_RANGE);
        topRightClaw.setPosition(RobotConstants.CLAW_MIN_RANGE);
        topLeftClaw.setPosition(RobotConstants.CLAW_MIN_RANGE);
        idolHand.setPosition(RobotConstants.IDOLHAND_MIN_RANGE);
        idolDead.setPosition(RobotConstants.IDOLHAND_MIN_RANGE);

    }
    
}
