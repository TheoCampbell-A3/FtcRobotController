/* Copyright (c) 2023 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@TeleOp(name="Main Driving Code (3.0)", group = "Concept")

public class tra3nrex_maindrv extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    private DcMotor pixelIntake      = null;
    private DcMotor pixelArm         = null;
    private Servo leftHand = null;
    private Servo rightHand = null;
    private DistanceSensor sensorDistance;
    private TouchSensor sensorTouch;
    boolean toggle = false;
    boolean bState = false;
    double pixelIntakePower = 0;
//    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
//    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
//    private VisionPortal visionPortal;               // Used to manage the video source.
//    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
//    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override public void runOpMode() {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        //initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        pixelIntake = hardwareMap.get(DcMotor.class, "toilet");
        pixelArm = hardwareMap.get(DcMotor.class, "flytrap");
        leftHand = hardwareMap.get(Servo.class, "left_hand");
        rightHand = hardwareMap.get(Servo.class, "right_hand");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        sensorTouch = hardwareMap.get(TouchSensor.class, "sensor_touch2");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        pixelIntake.setDirection(DcMotor.Direction.REVERSE);
        pixelArm.setDirection(DcMotor.Direction.FORWARD);

        //if (USE_WEBCAM)
        //    setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double leftHandPower = 1.0;
            double rightHandPower = 1.0;
            if (gamepad1.right_bumper && toggle == true) {
                leftHandPower = 0.4;
                toggle = false;
                bState = true;
            } else if (!gamepad1.right_bumper && toggle == true) {
                leftHandPower = 0.4;
                toggle = false;
                bState = false;
            } else if (gamepad1.right_bumper && toggle == false) {
                leftHandPower = 1.0;
                toggle = true;
                bState = true;
            } else if (gamepad1.right_bumper && toggle == false) {
                leftHandPower = 1.0;
                toggle = true;
                bState = false;
            }
            if (gamepad1.left_bumper && toggle == true) {
                rightHandPower = 0.4;
                toggle = false;
            } else if (gamepad1.left_bumper && toggle == false) {
                rightHandPower = 1.0;
                toggle = true;
            } else if (gamepad1.left_bumper && toggle == false) {
                rightHandPower = 1.0;
                toggle = true;
            } else if (gamepad1.left_bumper && toggle == false) {
                rightHandPower = 1.0;
                toggle = true;
            }
            if (gamepad1.b) {
                pixelIntakePower = 1;
            } else {
                pixelIntakePower = 0;
            }
            if (gamepad1.a) {
                pixelIntakePower = -1;
            } else {
                pixelIntakePower = 0;
            }
            leftHand.setPosition(leftHandPower);
            rightHand.setPosition(rightHandPower);

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_stick_button || gamepad1.right_stick_button) {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive = -gamepad1.left_stick_y / 1.0;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x / 1.0;  // Reduce strafe rate to 50%.
                turn = gamepad1.right_stick_x / 1.5;  // Reduce turn rate to 33%.
                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
                turn = gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
            if (sensorTouch.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x +y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x -y -yaw;
        double rightBackPower    =  x -y +yaw;
        double pixelArmPower     =  gamepad1.right_trigger*-.75+gamepad1.left_trigger*.75;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
            pixelIntakePower /= max;
            pixelArmPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        pixelIntake.setPower(pixelIntakePower);
        pixelArm.setPower(pixelArmPower);

    }


}
