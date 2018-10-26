package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoTeamBlueFoward", group="Pushbot")
//@Disabled
public class AutoBlueForward extends LinearOpMode {



    /* Declare OpMode members. */
    BotDawg         robot   = new BotDawg();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1.0;
    static final double     TURN_SPEED              = 0.5;
    VuforiaLocalizer vuforia;
    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //Vuforia



//        RelicRecoveryVuMark vuMark= null;
//        /*
//         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
//         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
//         */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        // OR...  Do Not Activate the Camera Monitor View, to save power
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        /*
//         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
//         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
//         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
//         * web site at https://developer.vuforia.com/license-manager.
//         *
//         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
//         * random data. As an example, here is a example of a fragment of a valid key:
//         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
//         * Once you've obtained a license key, copy the string from the Vuforia web site
//         * and paste it in to your code onthe next line, between the double quotes.
//         */
//        parameters.vuforiaLicenseKey = "Ac6qs5r/////AAAAmWu1qwHxmkGnhmkbgadu0MGDJ87NxMXmWpJoUadeNO2hgW+h1lJEa0bMLdo8tZAXqmr8W0Juur5LfWj0X7MC00ZL2zHFCiTHRpKJLp4/eKl4ga5cV4Hz98JwznZMsAqnSjrpHtp0axTMn8iAx4jVX6l1Caq6SPbGxZaQtbjxXobn2a1F+2NyyKS2Jdj4cJDlo/TPi+AFmQMQZB4XekZaDoaAHHe09W305YKXQ901TwvAkz9fiJoBcasK5C0y+5t/nJBTACkhFIWwpJJZErtyDLUjMNl/LoipKo0zkJjcjT8w7x8G4xslrJMDeD3sxrJd+yCcff8+APzsamhiOqj3mX1H0g2OndJAHvmO8bLjfRe1";
//
//        /*
//         * We also indicate which camera on the RC that we wish to use.
//         * Here we chose the back (HiRes) camera (for greater range), but
//         * for a competition robot, the front camera might be more convenient.
//         */
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//        /**
//         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
//         * in this data set: all three of the VuMarks in the game were created from this one template,
//         * but differ in their instance id information.
//         * @see VuMarkInstanceId
//         */
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//
//        telemetry.addData(">", "Press Play to start");
//        telemetry.update();
//        waitForStart();
//
//        relicTrackables.activate();
//
//
//
//            /**
//             * See if any of the instances of {@link relicTemplate} are currently visible.
//             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
//             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
//             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
//             */
//            vuMark = RelicRecoveryVuMark.from(relicTemplate);
//
//
//
//            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//
//                /* Found an instance of the template. In the actual game, you will probably
//                 * loop until this condition occurs, then move on to act accordingly depending
//                 * on which VuMark was visible. */
//                telemetry.addData("VuMark", "%s visible", vuMark);
//
//                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
//                 * it is perhaps unlikely that you will actually need to act on this pose information, but
//                 * we illustrate it nevertheless, for completeness. */
//                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
//                telemetry.addData("Pose", format(pose));
//
//                /* We further illustrate how to decompose the pose into useful rotational and
//                 * translational components */
//                if (pose != null) {
//                    VectorF trans = pose.getTranslation();
//                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
//                    double tX = trans.get(0);
//                    double tY = trans.get(1);
//                    double tZ = trans.get(2);
//
//                    // Extract the rotational components of the target relative to the robot
//                    double rX = rot.firstAngle;
//                    double rY = rot.secondAngle;
//                    double rZ = rot.thirdAngle;
//                }
//            }
//            else {
//                telemetry.addData("VuMark", "not visible");
//            }
//
//            telemetry.update();


        robot.init(hardwareMap);



        // Turn on the LED light for better accuracy of the scan
        robot.colorSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

//        robot.leftFrontMotor = hardwareMap.dcMotor.get("Leftfront");
//        robot.leftBackMotor = hardwareMap.dcMotor.get("Leftback");
//        robot.rightFrontMotor = hardwareMap.dcMotor.get("Rightfront");
//        robot.rightBackMotor = hardwareMap.dcMotor.get("Rightback");
//        robot.liftMotor = hardwareMap.dcMotor.get("Lift");
//        robot.leftClampServo = hardwareMap.servo.get("LeftClamp");
//        robot.rightClampServo = hardwareMap.servo.get("RightClamp");

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition(),
                robot.leftBackMotor.getCurrentPosition(),
                robot.rightBackMotor.getCurrentPosition());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Blue", robot.colorSensor.blue());
        telemetry.addData("Red", robot.colorSensor.red());


        armDown(2.5);//THIS WILL MOVE THE ARM WITH THE COLOR SENSOR DOWN
        jewel(2.5);//THIS WILL SCAN THE COLOR, DECIDE IN WHAT DIRECTION TO TURN, AND TURN

       // SafeZone(2.5, vuMark);

        encoderDrive(DRIVE_SPEED, -20, -20, 2.0);
        sleep(500);     // pause for servos to move
        encoderDrive(TURN_SPEED, 8,-8,2.0);
        sleep(500);     // pause for servos to move
        encoderDrive(DRIVE_SPEED, 2, 2, 2.0);

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();





    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void jewel(double holdTime){
        ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        holdTimer.reset();
        while(opModeIsActive() && holdTimer.time() < holdTime){
            if (robot.colorSensor.blue() > robot.colorSensor.red() + 2) {
                encoderDrive(DRIVE_SPEED, -2, -2, 2.0);
                robot.armServo.setPosition(1.0);
                encoderDrive(DRIVE_SPEED, 2, 2, 2.0);
            } else if (robot.colorSensor.blue() < robot.colorSensor.red() - 2) {
                encoderDrive(DRIVE_SPEED, 2, 2, 2.0);
                robot.armServo.setPosition(1.0);
                encoderDrive(DRIVE_SPEED, -2, -2, 2.0);
            } else {
                robot.leftBackMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);
                robot.rightBackMotor.setPower(0);
                robot.leftBackMotor.setPower(0);
            }
        }
        robot.armServo.setPosition(1.0);
    }
    public void armDown (double holdTime){
        ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        holdTimer.reset();
        while(opModeIsActive() && holdTimer.time() < holdTime){
            robot.armServo.setPosition(0.0);//IT WILL PUT THE ARM DOWN
        }
    }

    public void SafeZone (double holdtime, RelicRecoveryVuMark vuMark){
        ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        holdTimer.reset();
        // Depending on where we have to put the block it moves x inches.
        switch (vuMark){
            case LEFT:
                encoderDrive(DRIVE_SPEED, 22, 22, 2.0);
                encoderDrive(TURN_SPEED,-12,12,2.0);
                encoderDrive(DRIVE_SPEED, 22, 22, 2.0);
            case RIGHT:
                encoderDrive(DRIVE_SPEED, 40,40, 2.0);
                encoderDrive(TURN_SPEED,-12,12,2.0);
                encoderDrive(DRIVE_SPEED, 22, 22, 2.0);
            case CENTER:
                encoderDrive(DRIVE_SPEED, 36, 36, 2.0);
                encoderDrive(TURN_SPEED, -12,12,2.0);
                encoderDrive(DRIVE_SPEED, 22, 22, 2.0);
            case UNKNOWN:
                encoderDrive(DRIVE_SPEED, 26, 26, 2.0);
                encoderDrive(TURN_SPEED, -12,12,2.0);
                encoderDrive(DRIVE_SPEED, 22, 22, 2.0);
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
