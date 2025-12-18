package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

@Autonomous(name="BlueAuto2", group="LM1")
@Config
public class BlueAuto2 extends LinearOpMode  {

    private DcMotorEx shooterR;
    private DcMotorEx shooterL;
    private DcMotor intakeMotor;
    private DcMotor passer;
    private CRServo passerServo;

    AprilTagProcessor aprilTag;

    public static double SHOOTER_PID_P = 3.5;
    public static double SHOOTER_PID_I = 0.0;
    public static double SHOOTER_PID_D = 0.0;
    public static double SHOOTER_PID_F = 0.0;

    public static double SHOOTER_MOTOR_VELOCITY = 1500;

    public static double PASSER_WAIT_TIME = 2.5;

    public static Vector2d BLUE_SHOOT_LOCATION = new Vector2d(-18.0, -15.0);
    public static double BLUE_SHOOT_ANGLE =  Math.toRadians(43.0);

    public static double PASSER_SERVO_POWER = 1.0;
    public static double PASSER_MOTOR_POWER = -1.0;


    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-53.0, -53.0, Math.toRadians(45)));

        // Setup the intake motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intaker");

        // Setup the left shooter motor
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setup the right shooter motor
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setup the passer motor and servo
        passer = hardwareMap.get(DcMotor.class, "Mpasser");
        passerServo = hardwareMap.get(CRServo.class, "passer");

        initAprilTag();

        waitForStart();

        PIDFCoefficients leftShooterPID = new PIDFCoefficients(SHOOTER_PID_P, SHOOTER_PID_I, SHOOTER_PID_D, SHOOTER_PID_F);

        // Create a DcMotorControllerEx to get the PID set for the motor
        DcMotorControllerEx leftMotorControllerEx = (DcMotorControllerEx)shooterL.getController();

        // Get the index for the motor
        int leftMotorIndex = ((DcMotorEx)shooterL).getPortNumber();

        // Apply the PID to the motor controller
        leftMotorControllerEx.setPIDFCoefficients(leftMotorIndex, DcMotor.RunMode.RUN_USING_ENCODER, leftShooterPID);

        // Set the velocity for the shooter motors
        shooterL.setVelocity(SHOOTER_MOTOR_VELOCITY);
        shooterR.setVelocity(SHOOTER_MOTOR_VELOCITY);

        Actions.runBlocking(
                new ParallelAction(
                    drive.actionBuilder(new Pose2d(-53.0, -53.0, Math.toRadians(45)))
                        // Move to the shoot location
                        //.strafeToLinearHeading(new Vector2d(-18.0, -15.0), Math.toRadians(43.0))
                        .strafeToLinearHeading(BLUE_SHOOT_LOCATION, BLUE_SHOOT_ANGLE)
                        .stopAndAdd(new AimActionRed(drive))

                        // start to shoot
                        // Turn on the intake
                        .stopAndAdd(new IntakeAction(intakeMotor, false))
                        // Turn on the passer
                        .stopAndAdd(new PasserAction(passer, passerServo, false))
                        .waitSeconds(PASSER_WAIT_TIME)

                        // Turn off the passer
                        .stopAndAdd(new PasserAction(passer, passerServo, true))

                        // move to the first row
                        //.stopAndAdd(new IntakeAction(intakeMotor, false))
                        .strafeToLinearHeading(new Vector2d(-11.0, -16.0), Math.toRadians(90.0))
                        // move down
                        .strafeToLinearHeading(new Vector2d(-11.0, -52.0), Math.toRadians(90.0)) // y was originally 49. im testing it as 40
                        // move up
                        //.strafeToLinearHeading(new Vector2d(-11.0, -22.0), Math.toRadians(90.0))
                        // Turn off the intake
                        //.stopAndAdd(new IntakeAction(intakeMotor, true))

                        // Move to the shooting position
                        //.strafeToLinearHeading(new Vector2d(-18.0, -15.0), Math.toRadians(43.0))
                        .strafeToLinearHeading(BLUE_SHOOT_LOCATION, BLUE_SHOOT_ANGLE)
                        // Turn on the passer
                        .stopAndAdd(new PasserAction(passer, passerServo, false))
                        .waitSeconds(PASSER_WAIT_TIME)
                        // Turn off the passer
                        .stopAndAdd(new PasserAction(passer, passerServo, true))

                        // Move to the second row
                        //.stopAndAdd(new IntakeAction(intakeMotor, false))
                        .strafeToLinearHeading(new Vector2d(11.0, -22.0), Math.toRadians(90.0))
                        // move down
                        .strafeToLinearHeading(new Vector2d(11.0, -52.0), Math.toRadians(90.0))
                        // move up
                        //.strafeToLinearHeading(new Vector2d(11.0, -22.0), Math.toRadians(90.0))
                        // Turn off the intake
                        //.stopAndAdd(new IntakeAction(intakeMotor, true))

                        // Move to the shooting position
                        //.strafeToLinearHeading(new Vector2d(-18.0, -15.0), Math.toRadians(43.0))
                        .strafeToLinearHeading(BLUE_SHOOT_LOCATION, BLUE_SHOOT_ANGLE)

                        // Turn on the passer
                        .stopAndAdd(new PasserAction(passer, passerServo, false))
                        .waitSeconds(PASSER_WAIT_TIME)
                        // Turn off the passer
                        .stopAndAdd(new PasserAction(passer, passerServo, true))

                        // move to the third row
                        // Turn on the intake
                        //.stopAndAdd(new IntakeAction(intakeMotor, false))
                        .strafeToLinearHeading(new Vector2d(35.0, -18.0), Math.toRadians(90.0))
                        // move down
                        .strafeToLinearHeading(new Vector2d(34.0, -52), Math.toRadians(90.0))
                        // move up
                        //.strafeToLinearHeading(new Vector2d(34.0, -22.0), Math.toRadians(90.0))
                        // Turn off the intake
                        //.stopAndAdd(new IntakeAction(intakeMotor, true))

                        // Move to the shooting position
                        //.strafeToLinearHeading(new Vector2d(-18.0, -15.0), Math.toRadians(43.0))
                        .strafeToLinearHeading(BLUE_SHOOT_LOCATION, BLUE_SHOOT_ANGLE)

                        // Turn on the passer
                        .stopAndAdd(new PasserAction(passer, passerServo, false))
                        .waitSeconds(PASSER_WAIT_TIME)
                        // Turn off the passer
                        .stopAndAdd(new PasserAction(passer, passerServo, true))
                        .stopAndAdd(new IntakeAction(intakeMotor, true))
                        .build()
                    , new DebugShooterVelocity(shooterR, shooterL)
                )
        );

    }

    public class DebugShooterVelocity implements Action {

        DcMotorEx shooterR = null;
        DcMotorEx shooterL = null;

        DebugShooterVelocity(DcMotorEx shooterR, DcMotorEx shooterL) {
            this.shooterR = shooterR;
            this.shooterL = shooterL;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            telemetryPacket.put("Left Velocity", shooterL.getVelocity());
            telemetryPacket.put("Right Velocity", shooterR.getVelocity());
            return true;
        }
    }
    public class PasserAction implements Action {

        private DcMotor passerMotor;
        private CRServo passerServo;
        private boolean finished = false;
        public PasserAction( DcMotor passerMotor, CRServo passerServo, boolean finished) {
            this.passerMotor = passerMotor;
            this.passerServo = passerServo;
            this.finished = finished;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(finished) {
                passerMotor.setPower(0.0);
                passerServo.setPower(0.0);
            } else {
                passerMotor.setPower(PASSER_MOTOR_POWER);
                passerServo.setPower(PASSER_SERVO_POWER);
            }

            return false;
        }
    }

    public class IntakeAction implements Action {

        private DcMotor intakeMotor;
        private boolean finished = false;

        public IntakeAction( DcMotor intakeMotor, boolean finished) {
            this.intakeMotor = intakeMotor;
            this.finished = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(finished) {
                intakeMotor.setPower(0.0);
            } else {
                intakeMotor.setPower(.75);
            }

            return false;
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();
    }

    public class AimActionRed implements Action {

        MecanumDrive drive;
        private boolean keepRunning = true;
        List<AprilTagDetection> currentDetections;
        String currentDetectionsString = "";

        public AimActionRed(MecanumDrive drive) {
            this.drive = drive;

            Timer timer = new Timer();
            timer.schedule(new TimerTask() {

                @Override
                public void run() {
                    timeout();
                }
            }, 0, 1500);
        }

        public void timeout() {
            keepRunning = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Red 24
            // Blue 20
            //int targetId = 20;
            //int targetId = team;

            if(aprilTag == null) {
                // Apriltag is null, so return false
                return false;
            } else if(aprilTag.getDetections().isEmpty()) {

                // The camera does not see any tags so return false
                telemetryPacket.put("aprilTag", "No detections");
                return false;
            }

            // Get all of the detections from the camera
            currentDetections = aprilTag.getDetections();

            // Go through all of the april tag detections
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    currentDetectionsString = currentDetectionsString + " " + detection.id;

                    // Do we see the tag 20 (blue tag)?
                    if(detection.id == 20) {
                        if(detection.ftcPose.x < 0.5) {
                            Actions.runBlocking(drive.actionBuilder(new Pose2d(0,0,0))
                                    .turn(-0.5)
                                    .build());
                        } else if(detection.ftcPose.x > 0.5) {
                            Actions.runBlocking(drive.actionBuilder(new Pose2d(0,0,0))
                                    .turn(0.5)
                                    .build());
                        } else {
                            return keepRunning;
                        }
                    }
                }
            }

            return keepRunning;
        }
    }



    /// //////////////////////////////////////


}
