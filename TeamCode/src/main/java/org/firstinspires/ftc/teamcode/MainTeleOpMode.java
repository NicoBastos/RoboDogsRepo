package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BotDawg;

//import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.BotDawg;

@TeleOp(name = "org.firstinspires.ftc.teamcode.MainTeleOpMode", group = "TeleOp")

public class MainTeleOpMode extends OpMode {

    //Creating variables
    private ElapsedTime runtime = new ElapsedTime();

    double leftJoyStick, rightJoyStick;
    boolean g2X, g2B, g2Y, g2A;

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
        rightJoyStick = gamepad1.right_stick_x;
        g2X = gamepad2.x;
        g2B = gamepad2.b;
        g2Y = gamepad2.y;
        g2A = gamepad2.a;
        //This is for limiting the speed of the Lift motor if the driver wants to slow it down





        //Testing JOYSTICK_DEADBAND

        if (Math.abs(leftJoyStick) < JOYSTICK_DEADBAND){
            robot.leftBack.setPower(0);
            robot.leftFront.setPower(0);
        }

        if (Math.abs(rightJoyStick) < JOYSTICK_DEADBAND) {
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);
        }
        robot.rightFront.setPower(rightJoyStick);
        robot.leftFront.setPower(leftJoyStick);
        robot.rightBack.setPower(rightJoyStick);
        robot.leftBack.setPower(leftJoyStick);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "power (%.2f)", motorSpeed);
        // Controls for Latch Arm//
        if (g2A & !g2Y) {
            robot.liftMotor.setPower(-1);
        }
        else if (g2Y & !g2A) {
            robot.liftMotor.setPower(1);
        }



    }


    //use stop function to go back to bottom position
    //if (g2X & !g2B & (robot.latchServo.getPosition()) < 180) {//
    //robot.latchServo.setPosition(robot.latchServo.getPosition()+10);//
}//
//else if (g2B & !g2X & (robot.latchServo.getPosition()) > 0) {//
//robot.latchServo.setPosition(robot.latchServo.getPosition()-10);//
        //
                //telemetry.addData("CurrentPostition", "currentPosition: (%.2f)", liftUpdatedTicks);
