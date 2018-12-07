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

import com.disnodeteam.dogecv.math.MathFTC;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import android.util.Log;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
//import com.kauailabs.navx.ftc.AHRS;
//import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;


@Autonomous(name = "Depot", group = "Linear Opmode")

public class DepotAuto extends LinearOpMode {

    private final int NAVX_DIM_I2C_PORT = 0;
    NavxMicroNavigationSensor navx_device;
   // private navXPIDController yawPIDController;

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

//    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double MARKER_MOTOR_RUNTIME = 8500; // time to complete in milliseconds
    private final double SERVO_CYCLES = 25;
    private final long SERVO_SLEEP_TIME = 100;
//    private final double YAW_PID_P = 0.015; //0.005
//    private final double YAW_PID_I = 0.0;
//    private final double YAW_PID_D = 0.0;

    private final double TORQUENADO_TICKS_PER_REV = 1440;
    private final double WHEEL_CIRCUMFERENCE = 5 * Math.PI;

//    private boolean calibration_complete = false;



    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor leftMotor_front = null;
    private DcMotor rightMotor_front = null;

    private DigitalChannel touchSensor = null;

    private DcMotor elevator = null;
    private DcMotor markerPlacer = null;
    private Servo markerPlacementServo = null;
    private Servo samplingServo = null;

    private MarkerPlacer placer;

    private GoldAlignDetector detector;

    Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();
    DecimalFormat df;
    boolean robotAtBottom = false;


    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        navx_device = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        telemetry.addData("Loop", "Configured PID");


        df = new DecimalFormat("#.##");

        while (navx_device.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        markerPlacementServo.setPosition(0.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        resetAngle();
        sleep(200);

        double drive_speed = 1.0;


        /*******************************************
         ****** Lowering elevator code goes here ******
         ******************************************/

        lowerRobot(drive_speed / 2.0);
        double angleAtBottom = getCurrentAngle();
        idle();
        sleep(100);

        if (angleAtBottom > 3.0) {
            turnToDegrees(-angleAtBottom, drive_speed, 0.15);
            idle();
            sleep(500);
        }

        int gLoc = getGoldLocation();
        sleep(100);

        HashMap<Integer, String> loc = new HashMap<Integer, String>();
        loc.put(-1, "Left");
        loc.put(0, "Middle");
        loc.put(1, "Right");

        telemetry.addData("Gold Location", loc.get(gLoc));
        telemetry.update();

        sleep(300);

        turnToDegrees(-15, drive_speed * 2.0, 0.15);
        idle();
        sleep(250);
        driveToDistance(-5, -drive_speed, false);

        idle();
        sleep(250);

        turnToDegrees(15, drive_speed, 0);
        idle();
        sleep(100);

        driveToDistance(-8, -drive_speed, false);

        idle();
        sleep(100);

        placeMarker();
        idle();
        sleep(100);

        driveToDistance(6, drive_speed, false);
        idle();
        sleep(100);

        turnToDegrees(90, drive_speed, 0);
        idle();
        sleep(500);

        // Gold block is on the right
        if (gLoc == 1) {
            samplingServo.setPosition(0.0);
            idle();
            sleep(500);

            driveToDistance(15, drive_speed, false);
            idle();
            sleep(100);


            samplingServo.setPosition(0.6);
            sleep(100);

            driveToDistance(-60, -drive_speed, false);
            idle();
            sleep(100);

            // Gold block is on the left
        } else if (gLoc == -1) {
            driveToDistance(-12, -drive_speed, false);
            idle();
            sleep(100);

            samplingServo.setPosition(0.0);
            idle();
            sleep(1000);

            driveToDistance(-20, -drive_speed, false);
            idle();
            sleep(100);


            samplingServo.setPosition(0.6);
            sleep(100);

            driveToDistance(-13, -drive_speed, false);
            idle();
            sleep(100);


            // Gold location is either center or cannot be determined
        } else {
            samplingServo.setPosition(0.0);
            idle();
            sleep(1000);

            driveToDistance(-12, -drive_speed, false);
            idle();
            sleep(100);


            samplingServo.setPosition(0.6);
            sleep(100);

            driveToDistance(-33, -drive_speed, false);
            idle();
            sleep(100);

        }

        driveToDistance(-5, -drive_speed, false);
        idle();
        sleep(100);

        samplingServo.setPosition(0.6);



        // FOR TESTING

//        driveToDistance(15, drive_speed, false);
//        idle();
//        sleep(1000);
//        turnToDegrees(90, drive_speed);
//        idle();
//        sleep(1000);
//        driveToDistance(-15, -drive_speed, false);
//        idle();
//        sleep(1000);
//        turnToDegrees(-90, drive_speed);



    }

    double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.parseDouble(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }

    String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public void initializeHardware() {
        markerPlacementServo = hardwareMap.get(Servo.class, "marker_placement");
        placer = new MarkerPlacer(markerPlacementServo);
        samplingServo = hardwareMap.get(Servo.class, "sampling");
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor_front = hardwareMap.get(DcMotor.class, "left_drive_front");
        rightMotor_front = hardwareMap.get(DcMotor.class, "right_drive_front");
        leftMotor_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        markerPlacer = hardwareMap.get(DcMotor.class, "marker_placer");
        markerPlacer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        markerPlacementServo.setDirection(Servo.Direction.REVERSE);

        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor_front.setDirection(DcMotor.Direction.REVERSE);
        //rightDrive_front.setDirection(DcMotor.Direction.FORWARD);

//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftMotor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Settings for detecting gold block
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

    }

    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }



    public void driveToDistance(double inches, double speed, boolean dropMarker) {


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int ticks = (int) (inches * (TORQUENADO_TICKS_PER_REV / WHEEL_CIRCUMFERENCE));//(inches * 114.65);

        resetAngle();
        runtime.reset();

        while (Math.abs(leftMotor.getCurrentPosition()) < Math.abs(ticks)
                && Math.abs(rightMotor.getCurrentPosition()) < Math.abs(ticks)
                && Math.abs(leftMotor_front.getCurrentPosition()) < Math.abs(ticks)
                && Math.abs(rightMotor_front.getCurrentPosition()) < Math.abs(ticks)) {

            telemetry.addData("Target Position", ticks);
            telemetry.addData("Current Position", rightMotor.getCurrentPosition());
            telemetry.update();

            // Get current angle data and format yaw angle as current angle
            double currentAngle = getCurrentAngle();


            double correction = currentAngle * 0.05;

            if (dropMarker)
                placer.run();
            dropMarker = false;

            leftMotor.setPower(runtime.milliseconds() < 1000 ? limit(speed + correction) *
                    (runtime.milliseconds() / 1000.0) : limit(speed + correction));
            rightMotor.setPower(runtime.milliseconds() < 1000 ? limit(speed - correction) *
                    (runtime.milliseconds() / 1000.0) : limit(speed - correction));
            leftMotor_front.setPower(runtime.milliseconds() < 1000 ? limit(speed + correction) *
                    (runtime.milliseconds() / 1000.0) : limit(speed + correction));
            rightMotor_front.setPower(runtime.milliseconds() < 1000 ? limit(speed - correction) *
                    (runtime.milliseconds() / 1000.0) : limit(speed - correction));
        }

        rightMotor.setPower(0.0);
        leftMotor.setPower(0.0);
        rightMotor_front.setPower(0.0);
        leftMotor_front.setPower(0.0);


//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftMotor_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    public void turnToDegrees(double degrees, double speed, double baseSpeed) {


        double currentAngle = getCurrentAngle();
        double output = getRotationOutput(degrees, currentAngle);
        int lMotorDir = 0;

        if (currentAngle > degrees + TOLERANCE_DEGREES) {
            lMotorDir = 1;
        }

        if (currentAngle < degrees - TOLERANCE_DEGREES) {
            lMotorDir = -1;
        }

        runtime.reset();
        while ((currentAngle > degrees + TOLERANCE_DEGREES || currentAngle < degrees - TOLERANCE_DEGREES)
                && runtime.milliseconds() < 5000) {

            leftMotor.setPower(limit((lMotorDir * Math.min(output, speed) / 4.0) + (lMotorDir * baseSpeed)));
            leftMotor_front.setPower(limit((lMotorDir * Math.min(output, speed) / 4.0) + (lMotorDir * baseSpeed)));
            rightMotor.setPower(limit((-lMotorDir * Math.min(output, speed) / 4.0) + (-lMotorDir * baseSpeed)));
            rightMotor_front.setPower(limit((-lMotorDir * Math.min(output, speed) / 4.0) + (-lMotorDir * baseSpeed)));


            currentAngle = getCurrentAngle();
            telemetry.addData("Angle", getCurrentAngle());
            telemetry.update();
        }


        leftMotor.setPower(0.0);
        leftMotor_front.setPower(0.0);
        rightMotor.setPower(0.0);
        rightMotor_front.setPower(0.0);

//        navx_device.close();

    }

    public double getRotationOutput(double targetAngle, double currentAngle) {
        return 2 - (0.05 * (40 - Math.abs(targetAngle - currentAngle)));
    }

    public double getCurrentAngle() {
        // Get current angle data and format yaw angle as current angle
        angles = navx_device.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }

    public int getGoldLocation() {
        if (!detector.getAligned()) {
            if (detector.getXPosition() < 250) {
                return -1;
            } else {
                return 1;
            }
        }
        return 0;
    }

    public void doSampling(int gLoc) {

    }

    public void lowerRobot(double speed) {
        runtime.reset();
        while (!robotAtBottom && runtime.milliseconds() < 5000) {
            robotAtBottom = !touchSensor.getState();
            telemetry.addData("At Bottom", robotAtBottom);
            telemetry.update();
            elevator.setPower(speed);
            leftMotor.setPower(-speed / 2.0);
            rightMotor.setPower(-speed / 2.0);
        }
        elevator.setPower(0.0);
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        telemetry.addData("At Bottom", robotAtBottom);
        telemetry.update();
    }

    public void placeMarker() {
        markerPlacementServo.setPosition(0.3);
        samplingServo.setPosition(0.6);
        runtime.reset();
        while (runtime.milliseconds() < MARKER_MOTOR_RUNTIME) {
            markerPlacer.setPower(1.0);
        }
        markerPlacer.setPower(0.0);
        servoGoTo(markerPlacementServo, 1.0);
        samplingServo.setPosition(0.3);
        idle();
        sleep(1000);
        markerPlacementServo.setPosition(0.0);
    }

    public void servoGoTo(Servo servo, double position) {
        double startPos = servo.getPosition();
        double distanceToTarget = Math.abs(position - startPos);

        for (int i = 0; i < SERVO_CYCLES; i++) {
            servo.setPosition(startPos < position ? startPos + ((i + 1) * (distanceToTarget / SERVO_CYCLES)) :
                    startPos - ((i + 1) * (distanceToTarget / SERVO_CYCLES)));
            sleep(SERVO_SLEEP_TIME);
        }
    }

    public void resetAngle() {
        navx_device.resetDeviceConfigurationForOpMode();
    }
}

