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

package org.firstinspires.ftc.teamcode.pd;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pd.constants.RobotConstants;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareNut {

    /* Define Motor references ------------------------------------------------------------------*/
    private DcMotor  leftDrive   = null;        // Front left wheel
    private DcMotor  rightDrive  = null;        // Front right wheel
    private DcMotor  leftArm     = null;        // Back left wheel
    private DcMotor  rightArm    = null;        // Back right wheel
    private DcMotor  liftArm     = null;
    private DcMotor  ClawLift    = null;
    private DcMotor  ballarm     = null;
    private DcMotor  IdolSlide     = null;

    /* Define Servo references ------------------------------------------------------------------*/
    private Servo   leftClaw        = null;
    private Servo   rightClaw       = null;
    private Servo   topLeftClaw     = null;
    private Servo   topRightClaw    = null;
    private Servo   idolHand        = null;
    private Servo   IdolWrist        = null;

    /**
     * Creates an instance of the Hardware Nut class
     */
    public HardwareNut(){
    }
    /* Motor getters ----------------------------------------------------------------------------*/
    public DcMotor getLeftDrive() { return leftDrive; }

    public DcMotor getRightDrive() {
        return rightDrive;
    }

    public DcMotor getLeftArm() {
        return leftArm;
    }

    public DcMotor getRightArm() {
        return rightArm;
    }

    public DcMotor getLiftArm() {
        return liftArm;
    }

    public DcMotor getIdolSlide() {
        return IdolSlide;
    }

    public DcMotor getClawLift() {
        return ClawLift;
    }

    public DcMotor getBallarm() {
        return ballarm;
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

    public Servo getIdolWrist() { return IdolWrist;}

    public double[] getClawPositions() {
        return new double[]{leftClaw.getPosition(), rightClaw.getPosition(), topLeftClaw.getPosition(), topRightClaw.getPosition()};
    }

    public double[] getIdolHandPosition() {

       return new double[]{idolHand.getPosition(), IdolWrist.getPosition()};
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
        leftDrive  = hwMap.get(DcMotor.class, "left_Drive");
        rightDrive = hwMap.get(DcMotor.class, "right_Drive");
        leftArm    = hwMap.get(DcMotor.class, "left_Arm");
        rightArm   = hwMap.get(DcMotor.class, "right_Arm");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);  // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

  /*      *//* Arm Motors */
 /*       liftArm = hwMap.get(DcMotor.class, "lift_arm");
        ClawLift = hwMap.get(DcMotor.class, "claw_lift");
        ballarm = hwMap.get(DcMotor.class, "ball_arm");
        IdolSlide = hwMap.get(DcMotor.class, "idol_slide");
*/
        // Set all motors to zero power
        /* SET INITIAL POWER --------------------------------------------------------------------*/
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftArm.setPower(0);
        rightArm.setPower(0);
 /*       liftArm.setPower(0);
        ballarm.setPower(0);
        ClawLift.setPower(0);
        IdolSlide.setPower(0);
*/
        /* SET MOTOR MODE -----------------------------------------------------------------------*/
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  /*      liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ClawLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ballarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IdolSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/
 /*       *//* INITIALIZE SERVOS --------------------------------------------------------------------*//*
        topRightClaw  = hwMap.get(Servo.class, "top_right_claw");
        rightClaw  = hwMap.get(Servo.class, "right_claw");
        leftClaw = hwMap.get(Servo.class, "left_claw");
        topLeftClaw  = hwMap.get(Servo.class, "top_left_claw");
        idolHand = hwMap.get(Servo.class, "idol_hand");
        IdolWrist = hwMap.get(Servo.class, "idol_wrist");
*/
 /*       rightClaw.setPosition(RobotConstants.CLAW_MIN_RANGE);
        leftClaw.setPosition(RobotConstants.CLAW_MAX_RANGE);
        topRightClaw.setPosition(RobotConstants.CLAW_MIN_RANGE);
        topLeftClaw.setPosition(RobotConstants.CLAW_MIN_RANGE);
        idolHand.setPosition(RobotConstants.IDOLHAND_MIN_RANGE);
        IdolWrist.setPosition(RobotConstants.IDOLHAND_MIN_RANGE);
*/
    }
}