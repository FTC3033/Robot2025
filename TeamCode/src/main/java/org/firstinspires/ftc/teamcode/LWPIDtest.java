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


@Autonomous(name="LW PID shooter test", group="LM1")
public class LWPIDtest extends LinearOpMode  {
    public static double DISTANCE = 64;

    public int shootingIndex = 0;
    public int waitingIndex = 0;
    public int pidIndex = 0;
    public float shooterPower = 0.5F;
    private DcMotorEx shooterR;
    private DcMotorEx shooterL;
    private DcMotor intakeMotor;
    private DcMotor passer;
    private CRServo passerServo;

    public final double kP = 0.001;
    public final double kI = 0.00;
    public final double kD = 0.0;
    public final double TV = 2500;
    public double intsum = 0.0;
    public double lasterr = 0.0;
    public long lastUpdateTime = System.nanoTime();


    //private float time = 0.25F;
    private float time = 0.15F;
    private float stopTime = 1.75F; // a percentage (1.5 means 50% more) (bigger means more time for the motor to speed up, but takes up more time of auto)
    AprilTagProcessor aprilTag;

    //Red = 24, Blue = 20
    public static class team {
        private static int value = 24;

        public static int getValue() {
            return value;
        }

        public static void setValue(int value) {
            team.value = value;
        }
    };

    public boolean targetVelocity(float shooterSpeed, float shooterVelocity, int difference){
        // difference is the allowed range
        return ((((767.6767 * (Math.pow(shooterSpeed,2))) + (2967.6767 * shooterSpeed))>=shooterVelocity - difference))&&((((767.6767 * (Math.pow(shooterSpeed,2))) + (2967.6767 * shooterSpeed))<=shooterVelocity + difference));
    }

    public double PIDcon(double kp){
        return 4.2;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-52.0,47,43.05));

        intakeMotor = hardwareMap.get(DcMotor.class, "intaker");
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        passer = hardwareMap.get(DcMotor.class, "Mpasser");
        passerServo = hardwareMap.get(CRServo.class, "passer");
        passerServo.setDirection(DcMotorSimple.Direction.REVERSE);


        initAprilTag();

        waitForStart();
//        while(opModeIsActive()) {
//            long curenttime = System.nanoTime();
//            double dt = (curenttime - lastUpdateTime) / 1E9;
//            lastUpdateTime = curenttime;
//            double CV = shooterL.getVelocity();
//            double err = TV - CV;
//            double proportinal = kP * err;
//            intsum += err * dt;
//            double deriv = kD * (err - lasterr) / dt;
//            lasterr = err;
//            double outP = proportinal + (kI * intsum) + deriv;
//            double motorPower = Math.min(1.0, Math.max(-1.0, outP));
//
//            shooterL.setPower(motorPower);
//            shooterR.setPower(motorPower);
//            sleep(90);
//            passerServo.setPower(.9);
//            intakeMotor.setPower(.67);
//            passer.setPower(.5);
//        }

        // cant figure out a good way too set the velocity to the motorpower var

        /*
        I have an idea for that. Create a new public class that implements action. Then have it change the motor speed slowly until the motor speed reaches a calculation.
        For the calculation, you can use the target velocity function, but call it with the shooter speed as the power the motor should be, and the velocity of the motor currently
        While the motor speed is not within the allowed range, (targetVelocity returns false), change the power using either higher or lower. Call this class when you need
        to speed up or slow down the motor
                                                                -Nathan
        */

        Actions.runBlocking(drive.actionBuilder(new Pose2d(-52.0, 47.0, 43.05))
//                        //.lineToX(DISTANCE)
//                        //.splineTo(new Vector2d(DISTANCE, 0), Math.toRadians(180))
//                        //.splineToLinearHeading(new Pose2d(-3, 0, 0), Math.PI / 2)
//                        .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.75,false))
//                        .stopAndAdd(new PasserAction(passer, passerServo, false))
////                        .waitSeconds(2)
////                        .stopAndAdd(new PasserAction(passer, passerServo, true))
////                        .stopAndAdd(new ShooterAction(shooterR, shooterL, true))
////                        .lineToX(0)
//                        .stopAndAdd(new IntakeAction(intakeMotor, false))
//                        .waitSeconds(10)
//
////                        .waitSeconds(2)
////                        .stopAndAdd(new IntakeAction(intakeMotor, true))
                // move to the shoot point

                .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.70,false))
                .strafeToLinearHeading(new Vector2d(-23.0, 23.0), Math.toRadians(-43.0))
                // start to shoot the first three balls in pulses
                //.stopAndAdd(new ShooterAction(shooterR, shooterL, 0.45,false)) //testing 0

                .waitSeconds(0.09)
                .stopAndAdd(new PID())

//                .stopAndAdd(new WaitForSpeed(shooterL))
//                .stopAndAdd(new IntakeAction(intakeMotor, false))
//
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .stopAndAdd(new WaitForShoot(shooterL))
//                //.waitSeconds(time)
//                .stopAndAdd(new PasserAction(passer, passerServo, true))
//                //.waitSeconds(.25)
//                .stopAndAdd(new WaitForSpeed(shooterL))
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .stopAndAdd(new WaitForShoot(shooterL))
//                //.waitSeconds(time)
//                .stopAndAdd(new PasserAction(passer, passerServo, true))
//                //.waitSeconds(.25)
//                .stopAndAdd(new WaitForSpeed(shooterL))
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .stopAndAdd(new WaitForShoot(shooterL))
//                //.waitSeconds(time*2)
//                .stopAndAdd(new PasserAction(passer, passerServo, true))

                //.stopAndAdd(new PasserAction(passer, passerServo, true))
                //.waitSeconds(0.5)

                /*
                .waitSeconds(.03125)
                .stopAndAdd(new PasserAction(passer, passerServo, true))
                .waitSeconds(1)
                .stopAndAdd(new PasserAction(passer, passerServo, false))
                .waitSeconds(.03125)
                .stopAndAdd(new PasserAction(passer, passerServo, true))
                .waitSeconds(1)
                .stopAndAdd(new PasserAction(passer, passerServo, false))
                */

                //.waitSeconds(1)
                //  .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))


                // move to the first row
                .strafeToLinearHeading(new Vector2d(-11.0, 18.0), Math.toRadians(90.0))
                // move down
                .strafeToLinearHeading(new Vector2d(-11.0, 47.0), Math.toRadians(90.0)) // y was originally 49. im testing it as 40
                // move up
                .strafeToLinearHeading(new Vector2d(-11.0, 22.0), Math.toRadians(90.0))
                // move to the shoot point
                .strafeToLinearHeading(new Vector2d(-21.0, 21.0), Math.toRadians(-55.0)) //radians was -43. testing angles. Angle of -43 only messes up on this one
                //.strafeToLinearHeading(new Vector2d(-23,23), Math.toRadians(-55.0))
                .waitSeconds(0.09)
                .stopAndAdd(new PID())
                // start to shoot
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .stopAndAdd(new WaitForShoot(shooterL))
//                //.waitSeconds(time)
//                .stopAndAdd(new PasserAction(passer, passerServo, true))
//                .stopAndAdd(new WaitForSpeed(shooterL))
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .stopAndAdd(new WaitForShoot(shooterL))
//                //.waitSeconds(time)
//                .stopAndAdd(new PasserAction(passer, passerServo, true))
//                .stopAndAdd(new WaitForSpeed(shooterL))
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .stopAndAdd(new WaitForShoot(shooterL))
//                //.waitSeconds(time*2)
//                .stopAndAdd(new PasserAction(passer, passerServo, true))

                //  .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.55,false))
                //.waitSeconds(2)
                //.stopAndAdd(new IntakeAction(intakeMotor, false))
                //.stopAndAdd(new PasserAction(passer, passerServo, false))
                //.waitSeconds(1.2)
                //  .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))
                //.stopAndAdd(new PasserAction(passer, passerServo, true))

                // Move to the second row
                .strafeToLinearHeading(new Vector2d(11.0, 22.0), Math.toRadians(90.0))
                // move down
                .strafeToLinearHeading(new Vector2d(11.0, 40.0), Math.toRadians(90.0))
                // move up
                .strafeToLinearHeading(new Vector2d(11.0, 22.0), Math.toRadians(90.0))
                // move to the shoot point
                .strafeToLinearHeading(new Vector2d(-23.0, 23.0), Math.toRadians(-55.0))
                //.strafeToLinearHeading(new Vector2d(-23.0, 23.0), Math.toRadians(-55.0))
                .waitSeconds(0.09)
                .stopAndAdd(new PID())
                //shoot
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .stopAndAdd(new WaitForShoot(shooterL))
//                //.waitSeconds(time)
//                .stopAndAdd(new PasserAction(passer, passerServo, true))
//                .stopAndAdd(new WaitForSpeed(shooterL))
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .stopAndAdd(new WaitForShoot(shooterL))
//                //.waitSeconds(time)
//                .stopAndAdd(new PasserAction(passer, passerServo, true))
//                .stopAndAdd(new WaitForSpeed(shooterL))
//                .stopAndAdd(new PasserAction(passer, passerServo, false))
//                .stopAndAdd(new WaitForShoot(shooterL))
//                //.waitSeconds(time*2)
//                .stopAndAdd(new PasserAction(passer, passerServo, true))
                //  .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.55,false))
                //.waitSeconds(2)
                //.stopAndAdd(new IntakeAction(intakeMotor, false))
                //.stopAndAdd(new PasserAction(passer, passerServo, false))
                //.waitSeconds(1.2)
                // .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))
                //.stopAndAdd(new PasserAction(passer, passerServo, true))

                // move to the third row
                .strafeToLinearHeading(new Vector2d(35.0, 22.0), Math.toRadians(90.0))
                // move down
                .strafeToLinearHeading(new Vector2d(34.0, 47.0), Math.toRadians(90.0))
                // move up
                .strafeToLinearHeading(new Vector2d(34.0, 22.0), Math.toRadians(90.0))
                // move to the shoot point
                .strafeToLinearHeading(new Vector2d(-23.0, 23.0), Math.toRadians(-55.0))
                //shoot
                //  .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.55,false))
                //.waitSeconds(2)
                //.stopAndAdd(new IntakeAction(intakeMotor, false))
                .stopAndAdd(new PasserAction(passer, passerServo, false))
                .waitSeconds(1.2)
                .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.0,true))
                .stopAndAdd(new PasserAction(passer, passerServo, true))
                .stopAndAdd(new IntakeAction(intakeMotor, true))
                .build());


    }

    public class PID implements Action {

        int sleepTime = 75;
        int runTime = 1500;
        float idlePower = 0.6F;
        long curenttime = 0;
        double dt = 0;
        double CV = 0;
        double err = 0;
        double proportinal = 0;
        double deriv = 0;
        double outP = 0;
        double motorPower = 0;
        boolean con = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pidIndex++;
            curenttime = System.nanoTime();
            dt = (curenttime - lastUpdateTime) / 1E9;
            lastUpdateTime = curenttime;
            CV = shooterL.getVelocity();
            err = TV - CV;
            proportinal = kP * err;
            intsum += err * dt;
            deriv = kD * (err - lasterr) / dt;
            lasterr = err;
            outP = proportinal + (kI * intsum) + deriv;
            motorPower = Math.min(1.0, Math.max(-1.0, outP));

            shooterL.setPower(motorPower);
            shooterR.setPower(motorPower);

            sleep(sleepTime);
            passerServo.setPower(-1.5);
            intakeMotor.setPower(.85);
            passer.setPower(-.7);

            con = sleepTime*pidIndex>=runTime;
            if(con){
                pidIndex = 0;
                shooterL.setPower(idlePower);
                shooterR.setPower(idlePower);
                passerServo.setPower(0);
                //intakeMotor.setPower(0);
                passer.setPower(0);
            }
            return !con;
        }

    }
    public class WaitForSpeed implements Action {

        DcMotorEx shooterMotor;
        WaitForSpeed(DcMotorEx shooterMotor){
            this.shooterMotor = shooterMotor;
        }
        int mil = 100;
        int repTime = 1500;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            sleep(mil);
            waitingIndex++;
            boolean con = targetVelocity((float) this.shooterMotor.getPower(), (float) this.shooterMotor.getVelocity(), 10) || (waitingIndex>=repTime/mil);;
            if(con){
                waitingIndex = 0;
            }
            return !con;
        }
    }

    public class WaitForShoot implements Action { //
        DcMotorEx shooterMotor;
        int mil = 10;
        int repTime = 500; //Milliseconds


        WaitForShoot(DcMotorEx shooterMotor){
            this.shooterMotor = shooterMotor;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            sleep(mil);
            shootingIndex++;
            boolean con = !targetVelocity((float) this.shooterMotor.getPower(), (float) this.shooterMotor.getVelocity(), 20) || (shootingIndex>=repTime/mil);
            if(con){
                shootingIndex = 0;
            }
            return !con;
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
                passerMotor.setPower(1);
                passerServo.setPower(-1.5);
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
