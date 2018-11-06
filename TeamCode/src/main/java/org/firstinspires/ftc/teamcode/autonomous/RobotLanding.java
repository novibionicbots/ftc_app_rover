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

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.systems.RRVHardwarePushbot;

import java.util.List;

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

@Autonomous(name="Autonomous Landing using TensorFlow", group="Pushbot")
//@Disabled

public class RobotLanding extends LinearOpMode {

    /* Declare OpMode members. */
    RRVHardwarePushbot robot = new RRVHardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 8.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0.8188976 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private int goldMineralX = -1;
    private int silverMineral1X = -1;
    private int silverMineral2X = -1;

    Boolean detectedGoldMineral = Boolean.FALSE;

    private static final String VUFORIA_KEY = "ASZFNcz/////AAABmXbNftY1lUh2qVN06LaDcyk91NUp+t7H+AVib0mDkCZ1KThT2a8sIJCFaW7FUpNMTZ/FAKxTBEJ7AmeWc3eyzFYV2n6012K5BCd7fCsDrg2mHwQM8ckHRrR+FAQwnsp+28rXf+QJskCdGmNYnH2bvXyRBfHZzqrJQpedpogdhUZ4U16GqB2k+gDTYyAdZRb/Vj9StxN7HtF6I+8wZWG91Dh9+fL2geiWXzLYnHZjKcL7vYg2F1yit9/KT6480HiLnabp8DOCVsL2mI3TJE2pW9NoivUEyK71JzHIowBMsfUviZOWeIxoWfAqx8eYHpHjjbiFOV1xtfnVAmzWLxNUkM/7TBeohDgHNBbv48JtfbAo";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized robot");    //
        telemetry.update();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Step0",  "Starting at %7d",
                robot.rack_pinion.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDown(6.5);

        runtime.reset();

        //Unlatch
        while (opModeIsActive() && runtime.seconds()<0.5) {
            telemetry.addData("Step1","Unlatching....");
            telemetry.update();
            robot.setLeftRight(0.8, -0.3,0.8,-0.3);
        }
        robot.setLeftRight(0,0,0,0);
        Wait(1);

        robot.setLeftRight(-0.5,-0.5,-0.5,-0.5);
        Wait(0.75);
        robot.setLeftRight(0,0,0,0);

        //Start scanning
        useTensor();

        robot.setLeftRight(0,0,0,0);
        //        Wait(2);

//        Wait(2);
        //Commenting the 4 lines below for testing
//        robot.setLeftRight(0.2,-0.2,0.2,-0.2);
//        Wait(0.5);
//        robot.setLeftRight(0,0,0,0);
//        Wait(2);
//        telemetry.addData("X position of the cube", detector.getXPosition());
//        telemetry.update();


//        runtime.reset();

//        while (opModeIsActive() && runtime.seconds()<0.75) {
//            robot.setLeftRight(0, 0.8,0,0.8);
//        }
//        while (opModeIsActive()&& runtime.seconds() < 1){
//            robot.setLeftRight(-0.5,-0.5,-0.5,-0.5);
//        }
//
//
//        robot.setLeftRight(0,0,0,0);
        telemetry.addData("Is the cube detected?",detectedGoldMineral);
        telemetry.addData("Gold position",goldMineralX);
        telemetry.addData("Silver 1 position",silverMineral1X);
        telemetry.addData("Silver 2 position", silverMineral2X);
        telemetry.update();

        robot.setLeftRight(0,0,0,0);

        Wait(10);


    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDown(double inches) {
        int lastPos = robot.rack_pinion.getCurrentPosition();
        int finalPos = (int) (lastPos-COUNTS_PER_INCH*inches);

        while (opModeIsActive()&&robot.rack_pinion.getCurrentPosition()>finalPos) {
            robot.rack_pinion.setPower(-1.0);
        }

        robot.rack_pinion.setPower(0);
    }
    public void Wait(double seconds){
        runtime.reset();
        while(opModeIsActive() && runtime.seconds()< seconds){}
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void useTensor(){
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia();

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }

            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();
            waitForStart();

            if (opModeIsActive()) {
                /** Activate Tensor Flow Object Detection. */
                if (tfod != null) {
                    tfod.activate();
                }

                runtime.reset();

                while (opModeIsActive() && !detectedGoldMineral) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());

                            if (updatedRecognitions.size() > 0) {
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                        telemetry.addData("Gold Detected", true);
                                        detectedGoldMineral = Boolean.TRUE;
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                        telemetry.addData("Silver Detected", true);
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                        telemetry.addData("Silver Detected", true);
                                    }
                                }
                            }

                            telemetry.update();
                        }
                    }
                    else {
                        telemetry.addData("Problem:", "Could not initialize TensorFlow");
                        telemetry.update();
                    }

                    robot.setLeftRight(-0.05,0.05,-0.05,0.05);
                }

                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Left");
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Right");
                    } else {
                        telemetry.addData("Gold Mineral Position", "Center");
                    }
                }

                if (goldMineralX != -1 && silverMineral1X == -1 && silverMineral2X == -1) {
                        telemetry.addData("Gold Mineral Position", "Left");
                }

                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X == -1) {
                    telemetry.addData("Gold Mineral Position", "Center");
                }

                telemetry.update();

                runtime.reset();
                while(opModeIsActive() && runtime.seconds() < 5) {
                    robot.setLeftRight(-0.2,-0.2,-0.2,-0.2);
                    telemetry.addData("Moving to knock off", "");
                    telemetry.update();
                    if(tfod.getUpdatedRecognitions().isEmpty())break;
                }

            }

            if (tfod != null) {
                tfod.shutdown();
            }
        }
}