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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

@TeleOp(name="Drive", group="Iterative Opmode")
public class Drive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    private DcMotor elevator = null;
    private DcMotor markerPlacer = null;
    private Servo samplingServo = null;

    private DigitalChannel touchSensor = null;

    boolean isDriving = false;
    long beganDrive;
    long timeToMaxSpeed = 1000;
    boolean atBottom = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDriveFront  = hardwareMap.get(DcMotor.class, "left_drive_front");
        rightDriveFront = hardwareMap.get(DcMotor.class, "right_drive_front");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        markerPlacer = hardwareMap.get(DcMotor.class, "marker_placer");
        samplingServo = hardwareMap.get(Servo.class, "sampling");
        markerPlacer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        AndroidTextToSpeech tts;
        tts = new AndroidTextToSpeech();
        tts.initialize();
        tts.setLanguage("en");
        tts.speak("Oh my glumshanks");
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
        runtime.reset();
        gamepad1.refreshTimestamp();
        gamepad1.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        samplingServo.setPosition(0.6);

        atBottom = !touchSensor.getState();

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        double elevatorPower = -gamepad2.left_stick_y;



//        if ((leftPower > -0.1 && leftPower < 0.1) && (rightPower > -0.1 && rightPower < 0.1)) {
//            gamepad1.reset();
//            beganDrive = 0;
//            isDriving = false;
//        } else {
//            if (!isDriving)
//                isDriving = true;
//                beganDrive = (long)runtime.milliseconds();
//            gamepad1.setTimestamp((long) runtime.milliseconds());
//        }
//
//        double timeSinceMovementBegan = runtime.milliseconds() - beganDrive + 0.1;
//        if (timeSinceMovementBegan < timeToMaxSpeed) {
//            // Send calculated power to wheels
//            leftPower *= timeSinceMovementBegan / timeToMaxSpeed;
//            rightPower *= timeSinceMovementBegan / timeToMaxSpeed;
//        }
//
//        telemetry.addData("Motor speed LEFT", leftPower);

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        leftDriveFront.setPower(leftPower);
        rightDriveFront.setPower(rightPower);

//        if (atBottom && elevatorPower < 0) {
//            elevator.setPower(0.0);
//        } else {
//            elevator.setPower(elevatorPower / 2.0);
//        }

        elevator.setPower(elevatorPower / 2.0);

        if (gamepad2.a) {
            markerPlacer.setPower(1.0);
        } else if (gamepad2.b) {
            markerPlacer.setPower(-1.0);
        } else {
            markerPlacer.setPower(0.0);
        }



        double encoderValue = elevator.getCurrentPosition();


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Gamepad", "Timestamp: " + gamepad1.timestamp);
        telemetry.addData("Encoders", "Ticks: " + encoderValue);
        telemetry.addData("Eelvator", "At Bottom: " + touchSensor.getState());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}