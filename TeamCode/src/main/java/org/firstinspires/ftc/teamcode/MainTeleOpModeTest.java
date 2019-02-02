package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.BotDawg;

@TeleOp(name = "MainTeleOpModeTest", group = "TeleOp")

public class MainTeleOpModeTest extends OpMode {

    //Creating variables
    private ElapsedTime runtime = new ElapsedTime();

    double leftJoyStick, rightJoyStick;
    boolean g2X, g2B, g2Y, g2A;
    float g2RT, g2LT;

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



        //Testing JOYSTICK_DEADBAND

        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setPower(1);
        telemetry.addData("Robot Right Front", robot.rightFront.getCurrentPosition());
        telemetry.addData("Robot Lift Motor", robot.liftMotor.getCurrentPosition());
        telemetry.update();
//        robot.leftFront.setPower(-leftJoyStick);
//        robot.rightBack.setPower(-rightJoyStick);
//        robot.leftBack.setPower(-leftJoyStick);

//
        }
}




//    use stop function to go back to bottom position

        //
                //telemetry.addData("CurrentPostition", "currentPosition: (%.2f)", liftUpdatedTicks);
