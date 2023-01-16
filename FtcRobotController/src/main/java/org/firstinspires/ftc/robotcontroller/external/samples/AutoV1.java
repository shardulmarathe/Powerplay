package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
@Disabled
public class AutoWOSensor extends LinearOpMode{
  /* Declare OpMode members. */
  public DcMotor  frontleft = null;
  public DcMotor  frontright = null;
  public DcMotor  downleft = null;
  public DcMotor  downright = null;

  private ElapsedTime runtime = new ElapsedTime();

  static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
  static final double DRIVE_GEAR_REDUCTION = 0.585;     // Check this value with smart people
  static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
  static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
          (WHEEL_DIAMETER_INCHES * 3.14159265359);
  static final double DRIVE_SPEED = 0.6;
  static final double TURN_SPEED = 0.5;

  @Override
  public void runOpMode() {

    // Initialize the drive system variables.
    frontleft = hardwareMap.get(DcMotor.class, "leftfront");
    downleft = hardwareMap.get(DcMotor.class, "leftback");
    frontright = hardwareMap.get(DcMotor.class, "rightfront");
    downright = hardwareMap.get(DcMotor.class, "rightback");

    frontleft.setDirection(DcMotor.Direction.FORWARD);
    frontright.setDirection(DcMotor.Direction.REVERSE);
    downleft.setDirection(DcMotor.Direction.FORWARD);
    downright.setDirection(DcMotor.Direction.REVERSE);

    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    downleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    downright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    downleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    downright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
        telemetry.update();
        */

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    // we'll figure out timeouts afterward
    // strafe 2 feet left
    encoderStrafe(DRIVE_SPEED, 24, 24, 10.0);  // S1: Forward 47 Inches with 5 Sec timeout
    //move 2 feet forward
    encoderDrive(DRIVE_SPEED, 24, 24, 10.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
    // turn 90 degrees left
    encoderDrive(TURN_SPEED, -24, 24, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout
    //put the cone on high junction that we are facing
    int REPEAT = 1
  while REPEAT < 2
    // turn 90 right
    encoderDrive(TURN_SPEED, 24, -24, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout
    //move 1 foot forward
    encoderDrive(DRIVE_SPEED, 12, 12, 10.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
    //turn 90 right
    encoderDrive(TURN_SPEED, 24, -24, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout
    // move roughly 4.5 feet (until we are in range to grab from the stack)
    encoderDrive(DRIVE_SPEED, 54, 54, 15.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
    // for 2 times{
    REPEAT+1=REPEAT

    // grab cone

    // turn 180
    encoderDrive(TURN_SPEED, 48, -48, 15.0);  // S3: Reverse 24 Inches with 4 Sec timeout
    // move forward 3 feet
    encoderDrive(DRIVE_SPEED, 36, 36, 15.0);  // S2: Turn Right 12 Inches with 4 Sec timeout

    //turn 90 right
    encoderDrive(TURN_SPEED, 24, -24, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout
    // put cone

    // turn 90 back
    encoderDrive(TURN_SPEED, -24, 24, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout

    // move forward 3 feet
    encoderDrive(DRIVE_SPEED, 36, 36, 15.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
    //}


    //telemetry.addData("Path", "Complete");
    //telemetry.update();
    sleep(1000);  // pause to display final telemetry message.
  }

  /*
   *  Method to perform a relative move, based on encoder counts.
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
    int newLeftBackTarget;
    int newRightBackTarget;
    int newRightFrontTarget;

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

      // Determine new target position, and pass to motor controller
      newLeftFrontTarget = frontleft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
      newLeftBackTarget = downleft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
      newRightFrontTarget = frontright.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
      newRightBackTarget = downright.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
      frontleft.setTargetPosition(newLeftFrontTarget);
      downleft.setTargetPosition(newLeftBackTarget);
      frontright.setTargetPosition(newRightFrontTarget);
      downright.setTargetPosition(newRightBackTarget);

      // Turn On RUN_TO_POSITION
      frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      downleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      downright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset the timeout time and start motion.
      runtime.reset();
      frontleft.setPower(Math.abs(speed));
      frontright.setPower(Math.abs(speed));
      downleft.setPower(Math.abs(speed));
      downright.setPower(Math.abs(speed));

      // keep looping while we are still active, and there is time left, and both motors are running.
      // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
      // its target position, the motion will stop.  This is "safer" in the event that the robot will
      // always end the motion as soon as possible.
      // However, if you require that BOTH motors have finished their moves before the robot continues
      // onto the next step, use (isBusy() || isBusy()) in the loop test.
      while (opModeIsActive() &&
              (runtime.seconds() < timeoutS) &&
              (frontright.isBusy() && frontleft.isBusy() && downright.isBusy() && downleft.isBusy())) {

                /*
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
                 */
      }

      // Stop all motion;
      frontleft.setPower(0);
      frontright.setPower(0);
      downleft.setPower(0);
      downright.setPower(0);

      // Turn off RUN_TO_POSITION
      frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      downleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      downright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      sleep(250);   // optional pause after each move.
    }
  }

  public void encoderStrafe(double speed,
                            double leftInches, double rightInches,
                            double timeoutS) {
    int newLeftFrontTarget;
    int newLeftBackTarget;
    int newRightBackTarget;
    int newRightFrontTarget;

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

      // Determine new target position, and pass to motor controller
      newLeftFrontTarget = -(frontleft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
      newLeftBackTarget = (downleft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
      newRightFrontTarget = (frontright.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));
      newRightBackTarget = -(downright.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
      frontleft.setTargetPosition(newLeftFrontTarget);
      downleft.setTargetPosition(newLeftBackTarget);
      frontright.setTargetPosition(newRightFrontTarget);
      downright.setTargetPosition(newRightBackTarget);

      // Turn On RUN_TO_POSITION
      frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      downleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      downright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset the timeout time and start motion.
      runtime.reset();
      frontleft.setPower(Math.abs(speed));
      frontright.setPower(Math.abs(speed));
      downleft.setPower(Math.abs(speed));
      downright.setPower(Math.abs(speed));

      // keep looping while we are still active, and there is time left, and both motors are running.
      // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
      // its target position, the motion will stop.  This is "safer" in the event that the robot will
      // always end the motion as soon as possible.
      // However, if you require that BOTH motors have finished their moves before the robot continues
      // onto the next step, use (isBusy() || isBusy()) in the loop test.
      while (opModeIsActive() &&
              (runtime.seconds() < timeoutS) &&
              (frontright.isBusy() && frontleft.isBusy() && downright.isBusy() && downleft.isBusy())) {

                /*
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
                 */
      }

      // Stop all motion;
      frontleft.setPower(0);
      frontright.setPower(0);
      downleft.setPower(0);
      downright.setPower(0);

      // Turn off RUN_TO_POSITION
      frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      downleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      downright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      sleep(250);   // optional pause after each move.
    }
  }
}