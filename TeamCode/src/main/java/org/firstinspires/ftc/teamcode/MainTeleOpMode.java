package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BotDawg;

@TeleOp(name = "MainTeleOpMode", group = "TeleOp")

public class MainTeleOpMode extends OpMode {

    //Creating variables
    private ElapsedTime runtime = new ElapsedTime();

    double leftJoyStick, rightJoyStick;
    final private static double JOYSTICK_DEADBAND = 0.1;

    //Encoder Ticks Variables
    double motorSpeed = 1.0; //100%

    BotDawg robot;

    @Override
    public void init() {

        robot = new BotDawg();
        robot.init(hardwareMap);
    }

    //Code that resets the elapsed time once the driver hits play
    @Override
    public void start() {
        runtime.reset();
    }


    public void loop() {

        //Assigning gamepad values
        leftJoyStick = -gamepad1.left_stick_y;
        rightJoyStick = gamepad1.right_stick_y;
        //Testing JOYSTICK_DEADBAND

        if (Math.abs(leftJoyStick) < JOYSTICK_DEADBAND){
            robot.leftBack.setPower(0);
            robot.leftFront.setPower(0);
        }
        if (Math.abs(rightJoyStick) < JOYSTICK_DEADBAND) {
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);
        }

        //Setting power to the motors
        robot.rightFront.setPower(rightJoyStick);
        robot.leftFront.setPower(leftJoyStick);
        robot.rightBack.setPower(rightJoyStick);
        robot.leftBack.setPower(leftJoyStick);



        //Telemtry stuff
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "power (%.2f)", motorSpeed);
        //telemetry.addData("CurrentPostition", "currentPosition: (%.2f)", liftUpdatedTicks);

    }


    //use stop function to go back to bottom position
}