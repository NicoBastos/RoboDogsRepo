package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PushBot {

    public DcMotor motorleft = null;
    public DcMotor motorright = null;


    public PushBot(){

        }


        public void init(HardwareMap hardwareMap) {



            motorleft = hardwareMap.dcMotor.get("motorleft");
            motorright = hardwareMap.dcMotor.get("motorright");


            motorleft.setDirection(DcMotor.Direction.REVERSE);
            motorright.setDirection(DcMotor.Direction.FORWARD);

        }


    public void start() throws InterruptedException {
        motorleft.setPower(1);
        motorright.setPower(1);
        Thread.sleep(8000);


    }
}
