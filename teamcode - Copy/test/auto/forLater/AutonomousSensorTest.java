package org.firstinspires.ftc.teamcode.test.auto.forLater;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pd.HardwareNut;

import java.util.Locale;

/**
 * Created by afield on 9/27/2017.
 */
@Autonomous (name="Sensor Test", group="Test")
@Disabled
public class AutonomousSensorTest extends LinearOpMode {
    HardwareNut robot = new HardwareNut();
    SensorREVColorDistance hwSensor = new SensorREVColorDistance();
    private ElapsedTime runtime = new ElapsedTime();


    ColorSensor sensorColor;
    DistanceSensor sensorDistance;




    @Override
    public void runOpMode() {

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



                while (sensorColor.red() >= 65) {
                    robot.getLeftDrive().setPower(.5);
                    robot.getRightDrive().setPower(.5);
                    robot.getLeftArm().setPower(.5);
                    robot.getRightArm().setPower(.5);

               }
                while (sensorColor.blue() >= 65) {
                    robot.getLeftArm().setPower(-.5);
                    robot.getRightArm().setPower(-.5);
                    robot.getLeftDrive().setPower(-.5);
                    robot.getRightDrive().setPower(-.5);
                }
                if(sensorColor.red() < 65) {
                    robot.getLeftDrive().setPower(0);
                    robot.getRightDrive().setPower(0);
                    robot.getLeftArm().setPower(0);
                    robot.getRightArm().setPower(0);
                }
                else if (sensorColor.blue() < 65) {
                    robot.getLeftArm().setPower(0);
                    robot.getRightArm().setPower(0);
                    robot.getLeftDrive().setPower(0);
                    robot.getRightDrive().setPower(0);
                }



            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));





                }
            });


            telemetry.update();
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });



    }
}

