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

package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

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

@Autonomous(name="RedSideParkWC", group="Pushbot")
//@Disabled
public class RedSideAutoWC extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareWCamera robot   = new HardwareWCamera();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1538 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.328 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.6 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     DRIVE_SPEED2            = 0.45;
    static final double     DRIVE_SPEED3            = 0.3;
    static final double     DRIVE_SPEED4            = 0.75;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tage IDs of Cone
    int Zone1 = 13;
    int Zone2 = 14;
    int Zone3 = 15;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftfront.getCurrentPosition(),
                robot.rightfront.getCurrentPosition(),
                robot.leftback.getCurrentPosition(),
                robot.rightback.getCurrentPosition());
        telemetry.update();
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Zone1 || tag.id == Zone2 || tag.id == Zone3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
// Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // 12 is around 45 in turn
        // we'll figure out timeouts afterward
        // strafe 2 feet left

        if (tagOfInterest == null || tagOfInterest.id == Zone1){
            telemetry.addLine("Zone 1 (Should be tag 13)");
            telemetry.update();
            encoderDrive(DRIVE_SPEED,27.5,-27.5,-27.5,27.5,5.0);
            encoderDrive(DRIVE_SPEED3,-8,-8,-8,-8,5.0);

            //move 2 feet forward
            encoderDrive(DRIVE_SPEED,26.5,26.5,26.5,26.5,5.0);
            // turn 90 degrees left
            encoderDrive(DRIVE_SPEED3,12,-12,12,-12,5.0);
            // move towards high junction
            encoderDrive(DRIVE_SPEED3,6,6,6,6,5.0);
            //put the cone on high junction that we are facing
            Liftencoder(DRIVE_SPEED,-12, 2);
            encoderDrive(DRIVE_SPEED3,2,2,2,2,5.0);
            Liftencoder(DRIVE_SPEED3,0.5, 1);
            robot.grabber.setPosition(0.1);
            robot.grabber2.setPosition(1.0);
            encoderDrive(DRIVE_SPEED3,-2,-2,-2,-2,5.0);
            robot.Lift.setPower(0.0);


            encoderDrive(DRIVE_SPEED3,-8,-8,-8,-8,5.0);
            // turn 90 right
            encoderDrive(DRIVE_SPEED3,-11,11,-11,11,5.0);
        }
        else if (tagOfInterest.id == Zone2){
            telemetry.addLine("Zone 2 (Should be tag 14)");
            telemetry.update();
            encoderDrive(DRIVE_SPEED,27.5,-27.5,-27.5,27.5,5.0);
            encoderDrive(DRIVE_SPEED3,-8,-8,-8,-8,5.0);

            //move 2 feet forward
            encoderDrive(DRIVE_SPEED,26.5,26.5,26.5,26.5,5.0);
            // turn 90 degrees left
            encoderDrive(DRIVE_SPEED3,12,-12,12,-12,5.0);
            // move towards high junction
            encoderDrive(DRIVE_SPEED3,6,6,6,6,5.0);
            //put the cone on high junction that we are facing
            Liftencoder(DRIVE_SPEED,-12, 2);
            encoderDrive(DRIVE_SPEED3,2,2,2,2,5.0);
            Liftencoder(DRIVE_SPEED3,0.5, 1);
            robot.grabber.setPosition(0.1);
            robot.grabber2.setPosition(1.0);
            encoderDrive(DRIVE_SPEED3,-2,-2,-2,-2,5.0);
            robot.Lift.setPower(0.0);


            encoderDrive(DRIVE_SPEED3,-8,-8,-8,-8,5.0);
            // turn 90 right
            encoderDrive(DRIVE_SPEED3,-11,11,-11,11,5.0);
            encoderDrive(DRIVE_SPEED3,-21.5,21.5,-21.5,21.5,5.0);
            encoderDrive(DRIVE_SPEED3,18,18,18,18,5.0);
        }
        else if (tagOfInterest.id == Zone3){
            telemetry.addLine("Zone 3 (Should be tag 15)");
            telemetry.update();
                    //move 2 feet forward
            encoderDrive(DRIVE_SPEED,27.5,-27.5,-27.5,27.5,5.0);
            encoderDrive(DRIVE_SPEED3,-8,-8,-8,-8,5.0);

            //move 2 feet forward
            encoderDrive(DRIVE_SPEED,26.5,26.5,26.5,26.5,5.0);
            // turn 90 degrees left
            encoderDrive(DRIVE_SPEED3,12,-12,12,-12,5.0);
            // move towards high junction
            encoderDrive(DRIVE_SPEED3,6,6,6,6,5.0);
            //put the cone on high junction that we are facing
            Liftencoder(DRIVE_SPEED,-12, 2);
            encoderDrive(DRIVE_SPEED3,2,2,2,2,5.0);
            Liftencoder(DRIVE_SPEED3,0.5, 1);
            robot.grabber.setPosition(0.1);
            robot.grabber2.setPosition(1.0);
            encoderDrive(DRIVE_SPEED3,-2,-2,-2,-2,5.0);
            robot.Lift.setPower(0.0);


            encoderDrive(DRIVE_SPEED3,-8,-8,-8,-8,5.0);
            // turn 90 right
            encoderDrive(DRIVE_SPEED3,-11,11,-11,11,5.0);
            encoderDrive(DRIVE_SPEED3,-21.5,21.5,-21.5,21.5,5.0);
            encoderDrive(DRIVE_SPEED3,45,45,45,45,5.0);
        }
        sleep(10000);
//        encoderDrive(DRIVE_SPEED,30,-30,-30,30,5.0);
//        encoderDrive(DRIVE_SPEED3,-8,-8,-8,-8,5.0);
//
//        //move 2 feet forward
//        encoderDrive(DRIVE_SPEED,26.5,26.5,26.5,26.5,5.0);
//        // turn 90 degrees left
//        encoderDrive(DRIVE_SPEED3,11.5,-11.5,11.5,-11.5,5.0);
//        // move towards high junction
//        encoderDrive(DRIVE_SPEED3,7,7,7,7,5.0);
//        //put the cone on high junction that we are facing
//        Liftencoder(DRIVE_SPEED,-12, 2);
//        encoderDrive(DRIVE_SPEED3,2.5,2.5,2.5,2.,5.0);
//        Liftencoder(DRIVE_SPEED3,0.5, 1);
//        robot.grabber.setPosition(0.1);
//        robot.grabber2.setPosition(1.0);
//        encoderDrive(DRIVE_SPEED3,-2,-2,-2,-2,5.0);
//        robot.Lift.setPower(0.0);
//
//
//        encoderDrive(DRIVE_SPEED3,-8,-8,-8,-8,5.0);
//        // turn 90 right
//        encoderDrive(DRIVE_SPEED3,-12,12,-12,12,5.0);
////        encoderDrive(DRIVE_SPEED,-26.5,-26.5,-26.5,-26.5,5.0);
//        encoderDrive(DRIVE_SPEED3,3,3,3,3,5.0);
//
//        encoderDrive(DRIVE_SPEED,-45,45,45,-45,5.0);


        //move 1 foot forward

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
                             double frontleftInches, double frontrightInches,double downleftInches, double downrightInches,
                             double timeoutS) {
        int newfrontleftTarget;
        int newfrontrightTarget;
        int newdownleftTarget;
        int newdownrightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newfrontleftTarget = robot.leftfront.getCurrentPosition() + (int)(frontleftInches * COUNTS_PER_INCH);
            newfrontrightTarget = robot.rightfront.getCurrentPosition() + (int)(frontrightInches * COUNTS_PER_INCH);
            newdownleftTarget = robot.leftback.getCurrentPosition() + (int)(downleftInches * COUNTS_PER_INCH);
            newdownrightTarget = robot.rightback.getCurrentPosition() + (int)(downrightInches * COUNTS_PER_INCH);
            robot.leftfront.setTargetPosition(newfrontleftTarget);
            robot.rightfront.setTargetPosition(newfrontrightTarget);
            robot.leftback.setTargetPosition(newdownleftTarget);
            robot.rightback.setTargetPosition(newdownrightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftfront.setPower(Math.abs(speed));
            robot.rightfront.setPower(Math.abs(speed));
            robot.leftback.setPower(Math.abs(speed));
            robot.rightback.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftfront.isBusy() && robot.rightfront.isBusy()
                            && robot.leftback.isBusy() && robot.rightback.isBusy()))

            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newfrontleftTarget,
                        newfrontrightTarget,newdownleftTarget,newdownrightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftfront.getCurrentPosition(),
                        robot.rightfront.getCurrentPosition(),
                        robot.leftback.getCurrentPosition(),
                        robot.rightback.getCurrentPosition());
                telemetry.update();
            }



            // Stop all motion;
            robot.leftfront.setPower(0);
            robot.rightfront.setPower(0);
            robot.leftback.setPower(0);
            robot.rightback.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
            //hello
        }

    }
    public void Liftencoder(double speed, double Liftinches, double timeoutS) {
        int newLiftTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newLiftTarget = robot.Lift.getCurrentPosition() + (int)(Liftinches * COUNTS_PER_INCH);

            robot.Lift.setTargetPosition(newLiftTarget);


            // Turn On RUN_TO_POSITION
            robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.Lift.setPower(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.Lift.isBusy()))

            {

                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLiftTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d", robot.Lift.getCurrentPosition());
//                telemetry.update();
            }



            // Stop all motion;
            robot.Lift.setPower(0);



            // Turn off RUN_TO_POSITION
            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            //  sleep(250);   // optional pause after each move
            //hello
        }

    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


}
