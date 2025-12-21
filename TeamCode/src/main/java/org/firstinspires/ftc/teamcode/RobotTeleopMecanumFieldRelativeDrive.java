/* Copyright (c) 2025 FIRST. All rights reserved.
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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 *
 */
@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
@Config
//public class RobotTeleopMecanumFieldRelativeDrive extends OpMode {
public class RobotTeleopMecanumFieldRelativeDrive extends LinearOpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    boolean slow = false;
    boolean norm = false;
    int reverse = 1;

    DcMotor intakeMotor;
    DcMotorEx shooterL;
    DcMotorEx shooterR;
    DcMotor passer;
    CRServo passerServo;

    VisionPortal.Builder builder;
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;

    //final CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();
    OpenCvWebcam openCvWebcam;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    public static int TARGET_LEFT = 220;
    public static int TARGET_RIGHT = 280;
    public static double TARGET_ROTATE_SPEED = 0.25;

    public static double SHOOTER_PID_P = 3.5;
    public static double SHOOTER_PID_I = 0.0;
    public static double SHOOTER_PID_D = 0.0;
    public static double SHOOTER_PID_F = 0.0;
    public static double SHOOTER_MOTOR_VELOCITY_CLOSE = 1500;
    public static double SHOOTER_MOTOR_VELOCITY_FAR = 1500;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backL");
        backRightDrive = hardwareMap.get(DcMotor.class, "backR");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        intakeMotor = hardwareMap.get(DcMotor.class, "intaker");
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        passer = hardwareMap.get(DcMotor.class, "Mpasser");
        passerServo = hardwareMap.get(CRServo.class, "passer");

        //final CameraStreamProcessor processor = new CameraStreamProcessor();

        /*new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();*/

        //FtcDashboard.getInstance().startCameraStream(processor, 0);

        /*VisionPortal.Builder builder = new VisionPortal.Builder();
        //WebcamName webcamname = hardwareMap.get(WebcamName.class, "Webcam 1");
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        // Build the Vision Portal, using the above settings.
        VisionPortal visionPortal = builder.build();*/

        initAprilTag();
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();

        ///////////

        PIDFCoefficients leftShooterPID = new PIDFCoefficients(SHOOTER_PID_P, SHOOTER_PID_I, SHOOTER_PID_D, SHOOTER_PID_F);

        // Create a DcMotorControllerEx to get the PID set for the motor
        DcMotorControllerEx leftMotorControllerEx = (DcMotorControllerEx)shooterL.getController();

        // Get the index for the motor
        int leftMotorIndex = ((DcMotorEx)shooterL).getPortNumber();

        // Apply the PID to the motor controller
        leftMotorControllerEx.setPIDFCoefficients(leftMotorIndex, DcMotor.RunMode.RUN_USING_ENCODER, leftShooterPID);

        // Set the velocity for the shooter motors
        //shooterL.setVelocity(SHOOTER_MOTOR_VELOCITY);
        //shooterR.setVelocity(SHOOTER_MOTOR_VELOCITY);

        ///////////

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addLine("Press B to reset Yaw");

            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();

            //FtcDashboard.getInstance().sendImage(cameraStreamProcessor.get);
            //FtcDashboard.getInstance().sendImage(cameraStreamProcessor);
            //FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor, 0);

            //visionPortal.resumeStreaming();

            // If you press the A button, then you reset the Yaw to be zero from the way
            // the robot is currently pointing
            if (gamepad1.b) {
                imu.resetYaw();
            }

            if (gamepad1.left_trigger > .2) {
                reverse = -1;
            } else {
                reverse = 1;
            }

            if (gamepad1.y) {
                // Red 24
                // Blue 20
                if (aimAtTarget(24)) {
                    telemetry.addLine("aimAtTarget returned true");
                } else {
                    telemetry.addLine("aimAtTarget returned false");
                }

            }

            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.left_trigger > .2 || reverse < 0) {
                intakeMotor.setPower(.75 * reverse);
                //passer.setPower(.8 * reverse);
            } else {
                //passer.setPower(0);
                intakeMotor.setPower(0);
            }

            // booleans for shooters speed

            /*
            if(gamepad1.right_trigger > .2 && gamepad1.b) {
                shooterL.setPower(.7);
                shooterR.setPower(.7);
            } else if(gamepad1.right_trigger > .2 && gamepad1.a) {
                shooterL.setPower(.5);
                shooterR.setPower(.5);
            } else if(gamepad1.right_trigger > .2) {
                shooterL.setPower(.65);
                shooterR.setPower(.65);
            } else {
                shooterL.setPower(0);
                shooterR.setPower(0);
            }
            */
            //set shooters speed
            if (gamepad1.a) {
                norm = !norm;
                slow = false;
                sleep(200);
            } else if (gamepad1.x) {
                norm = false;
                slow = !slow;
                sleep(200);
            }

            //remove fast mode
            if (norm) {
                //shooterL.setPower(.5); //1400
                //shooterR.setPower(.5);
                shooterL.setVelocity(SHOOTER_MOTOR_VELOCITY_CLOSE);
                shooterR.setVelocity(SHOOTER_MOTOR_VELOCITY_CLOSE);
            } else if (slow) {
                //shooterL.setPower(.67); //1180
                //velocity = 767.676767x^2*2967.67676767x where x is power
                //shooterR.setPower(.67);
                shooterL.setVelocity(SHOOTER_MOTOR_VELOCITY_FAR);
                shooterR.setVelocity(SHOOTER_MOTOR_VELOCITY_FAR);
            } else {
                shooterR.setPower(0);
                shooterL.setPower(0);
            }
            if (shooterR.getPower() > 0 && shooterL.getPower() > 0) {
                telemetry.addLine("Shooter Power = " + ((shooterR.getPower() + shooterL.getPower()) / 2));
                telemetry.addLine("Shooter Speed = " + shooterL.getVelocity());
                telemetry.addLine("Shooter Target Speed?:" + ((((767.6767 * (Math.pow(shooterL.getPower(), 2))) + (2967.6767 * shooterL.getPower())) >= shooterL.getVelocity() - 40)));
            }

            if (gamepad1.right_bumper) {
                passer.setPower(-.7 * reverse);
                passerServo.setPower(1 * reverse);
            } else {
                passer.setPower(0);
                passerServo.setPower(0);
            }
//            if(gamepad1.dpad_down && gamepad1.right_bumper) {
//                passer.setPower(-1);
//                passerServo.setPower(-1);
//
//            }
        }

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        //if (USE_WEBCAM) {
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        //} else {
        //    builder.setCamera(BuiltinCameraDirection.BACK);
        //}

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);


    }

    public boolean aimAtTarget(int targetId) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            //if (detection.metadata != null) {
                //if (detection.id == targetId) {
                    if (detection.center.x < TARGET_LEFT) {
                        drive(0.0, 0.0, -TARGET_ROTATE_SPEED);
                    } else if (detection.center.x > TARGET_RIGHT) {
                        drive(0.0, 0.0, TARGET_ROTATE_SPEED);
                    } else {
                        return true;
                    }
                //}
            //}
        }

        return false;
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            /*if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));


            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }*/

            telemetry.addLine(String.format("\n==== (ID %d)", detection.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
}
