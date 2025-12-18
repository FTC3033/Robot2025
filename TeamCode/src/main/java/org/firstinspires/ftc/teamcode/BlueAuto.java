package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="Blue-Auto", group="LM1")
public class BlueAuto extends LinearOpMode  {
    public static double DISTANCE = 64;

    private DcMotorEx shooterR;
    private DcMotorEx shooterL;
    private DcMotor intakeMotor;
    private DcMotor passer;
    private CRServo passerServo;

    //private float time = 0.25F;
    private float time = 0.15F;
    private float stopTime = 1.75F; // a percentage (1.5 means 50% more) (bigger means more time for the motor to speed up, but takes up more time of auto)
    AprilTagProcessor aprilTag;

    //Red = 24, Blue = 20
    public static class team {
        private static int value = 20;

        public static int getValue() {
            return value;
        }

        public static void setValue(int value) {
            team.value = value;
        }
    };

    public boolean targetVelocity(float shooterSpeed, float shooterVelocity){
        int difference = 40; //The allowed range
        return ((((767.6767 * (Math.pow(shooterSpeed,2))) + (2967.6767 * shooterSpeed))>=shooterVelocity - difference));
    }

    @Override
    public void runOpMode() throws InterruptedException {
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-18,-22,215));
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        intakeMotor = hardwareMap.get(DcMotor.class, "intaker");
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        passer = hardwareMap.get(DcMotor.class, "Mpasser");
        passerServo = hardwareMap.get(CRServo.class, "passer");

        initAprilTag();

        waitForStart();


        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
//                .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.54,false))
//
//                .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(225))
//                // start to shoot
//                .waitSeconds(1.3)
//                .stopAndAdd(new IntakeAction(intakeMotor, false))
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//
//                .waitSeconds(1.5)
//                //  .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))
//                .stopAndAdd(new PasserAction(passer, passerServo, true))
//
//                // move to the first row
//                .strafeToLinearHeading(new Vector2d(-60, -48), Math.toRadians(0))
//                // move down
//                .strafeToLinearHeading(new Vector2d(-60, -20), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(225))
//
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .waitSeconds(1.2)
//                //  .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))
//                .stopAndAdd(new PasserAction(passer, passerServo, true))
//                // Move to the second row
//                .strafeToLinearHeading(new Vector2d(-84, -48), Math.toRadians(0))
//                // move down
//                .strafeToLinearHeading(new Vector2d(-84, -20), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(225))
//
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .waitSeconds(1.2)
//                // .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))
//                .stopAndAdd(new PasserAction(passer, passerServo, true))
//                // move to the third row
//                .strafeToLinearHeading(new Vector2d(-108, -48), Math.toRadians(0))
//                // move down
//                .strafeToLinearHeading(new Vector2d(-108, -20), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(225))
//
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .waitSeconds(1.2)
//                .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))
//                .stopAndAdd(new PasserAction(passer, passerServo, true))
//                .stopAndAdd(new IntakeAction(intakeMotor, true))

                 //move to the shoot point (old)
                .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.5,false))

                .strafeToLinearHeading(new Vector2d(63, -8), Math.toRadians(-5))
                // start to shoot
                .waitSeconds(1.3)
                .stopAndAdd(new IntakeAction(intakeMotor, false))
                .stopAndAdd(new PasserAction(passer, passerServo, false))

                .waitSeconds(1.3)
                //  .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))
                .stopAndAdd(new PasserAction(passer, passerServo, true))

                // move to the first row
                .strafeToLinearHeading(new Vector2d(50.0, -21.0), Math.toRadians(-140.0))
                // move down
                .strafeToLinearHeading(new Vector2d(27, -36.7), Math.toRadians(-140))
                .strafeToLinearHeading(new Vector2d(63,-8), Math.toRadians(-5))
                .stopAndAdd(new PasserAction(passer, passerServo, false))
                .waitSeconds(1.25)
                //  .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))
                .stopAndAdd(new PasserAction(passer, passerServo, true))
                // Move to the second row
                .strafeToLinearHeading(new Vector2d(65, -37.0), Math.toRadians(-140.0))
                // move down
                .strafeToLinearHeading(new Vector2d(41.3, -53.2), Math.toRadians(-140))
                .strafeToLinearHeading(new Vector2d(63,-8), Math.toRadians(-5))
                .stopAndAdd(new PasserAction(passer, passerServo, false))
                .waitSeconds(1.2)
                // .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))
                .stopAndAdd(new PasserAction(passer, passerServo, true))
                // move to the third row
                .strafeToLinearHeading(new Vector2d(79, -58), Math.toRadians(-140))
                // move down
                .strafeToLinearHeading(new Vector2d(56, -79), Math.toRadians(-140))
                .strafeToLinearHeading(new Vector2d(63,-8), Math.toRadians(-5))
                //.stopAndAdd(new PasserAction(passer, passerServo, false))
                .waitSeconds(1.2)
                .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))
                .stopAndAdd(new PasserAction(passer, passerServo, true))
                .stopAndAdd(new IntakeAction(intakeMotor, true))
                .build());

    }

    public class WaitForSpeed implements Action {

        DcMotorEx shooterMotor;
        WaitForSpeed(DcMotorEx shooterMotor){
            this.shooterMotor = shooterMotor;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            sleep(10);
            return !targetVelocity((float) this.shooterMotor.getPower(), (float) this.shooterMotor.getVelocity());
        }
    }
    public class ShooterAction implements Action {

        DcMotorEx shooterMotor1;
        DcMotorEx shooterMotor2;

        private double velocity;
        //private final int P = 2;
        //private final int ticks = 28;
        boolean finished = false;

        ShooterAction(DcMotorEx shooterMotor1, DcMotorEx shooterMotor2, double velocity, boolean finished) {
            this.shooterMotor1 = shooterMotor1;
            this.shooterMotor2 = shooterMotor2;
            this.finished = finished;
            this.velocity = velocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(finished) {
                shooterMotor1.setPower(0.0);
                shooterMotor2.setPower(0.0);
            } else {
                //shooterMotor1.setPower(P*(Math.abs(velocity*2800-shooterMotor1.getVelocity())/(velocity*28)));
                //shooterMotor2.setPower(P*(Math.abs(velocity*2800-shooterMotor2.getVelocity())/(velocity*28)));
                //double vel = shooterMotor1.getVelocity();//2800 usually
                //telemetryPacket.put("V1", vel);
                shooterMotor1.setPower(velocity);
                shooterMotor2.setPower(velocity);
            }

            return false;
        }
    }

    /*public class AimActionRed implements Action {

        MecanumDrive drive;

        public AimActionRed(MecanumDrive drive) {
            this.drive = drive;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Red 24
            // Blue 20
            //int targetId = 20;
            int targetId = team;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if(detection.id == targetId) {
                        if(detection.ftcPose.x < 0.5) {
                            Actions.runBlocking(drive.actionBuilder(new Pose2d(0,0,0))
                                    .turn(-0.5)
                                    .build());
                        } else if(detection.ftcPose.x > 0.5) {
                            Actions.runBlocking(drive.actionBuilder(new Pose2d(0,0,0))
                                    .turn(0.5)
                                    .build());
                        } else {
                            return true;
                        }
                    }
                }
            }

            return false;
        }
    }*/

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
                passerMotor.setPower(-1);
                passerServo.setPower(1.5);
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
    }   // end method initAprilTag()
}
