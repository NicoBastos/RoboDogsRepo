package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
@Disabled
public class JoystickTest extends OpMode
{
    BotDawg robot;

    double rightStick;
    final private static double JOYSTICK_DEADBAND = 0.1;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

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

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        rightStick = -gamepad1.right_stick_y;
        if (Math.abs(rightStick) < JOYSTICK_DEADBAND) {
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
        }
        robot.rightFront.setPower(rightStick);
        robot.leftFront.setPower(rightStick);
        robot.rightBack.setPower(rightStick);
        robot.leftBack.setPower(rightStick);



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
