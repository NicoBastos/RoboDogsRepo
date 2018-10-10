import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        //leftJoyStick = -gamepad1.left_stick_y;
       // rightJoyStick = gamepad1.right_stick_x;




        //This is for limiting the speed of the Lift motor if the driver wants to slow it down.







        //Testing JOYSTICK_DEADBAND

        //if (Math.abs(leftJoyStick) < JOYSTICK_DEADBAND) leftJoyStick = 0;
        //if (Math.abs(rightJoyStick) < JOYSTICK_DEADBAND) rightJoyStick = 0;

        robot.motor.setPower(motorSpeed);



        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "power (%.2f)", motorSpeed);
        //telemetry.addData("CurrentPostition", "currentPosition: (%.2f)", liftUpdatedTicks);

    }


    //use stop function to go back to bottom position
}