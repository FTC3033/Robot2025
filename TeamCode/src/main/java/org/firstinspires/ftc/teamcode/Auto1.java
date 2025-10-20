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

@Autonomous(name="Auto1", group="Robot")
public class Auto1 extends LinearOpMode  {
    public static double DISTANCE = 64;

    private DcMotorEx shooterR;
    private DcMotorEx shooterL;
    private DcMotor intakeMotor;
    private DcMotor passer;
    private CRServo passerServo;



    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        intakeMotor = hardwareMap.get(DcMotor.class, "intaker");
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        passer = hardwareMap.get(DcMotor.class, "Mpasser");
        passerServo = hardwareMap.get(CRServo.class, "passer");

        waitForStart();

        Actions.runBlocking(drive.actionBuilder(new Pose2d(0,0,0))
                        //.lineToX(DISTANCE)
                        //.splineTo(new Vector2d(DISTANCE, 0), Math.toRadians(180))
                        //.splineToLinearHeading(new Pose2d(-3, 0, 0), Math.PI / 2)
                        .stopAndAdd(new ShooterAction(shooterR, shooterL, 0.75,false))
                        .stopAndAdd(new PasserAction(passer, passerServo, false))
//                        .waitSeconds(2)
//                        .stopAndAdd(new PasserAction(passer, passerServo, true))
//                        .stopAndAdd(new ShooterAction(shooterR, shooterL, true))
//                        .lineToX(0)
                        .stopAndAdd(new IntakeAction(intakeMotor, false))
                        .waitSeconds(10)

//                        .waitSeconds(2)
//                        .stopAndAdd(new IntakeAction(intakeMotor, true))
                        .build());

    }

    public class ShooterAction implements Action {

        DcMotorEx shooterMotor1;
        DcMotorEx shooterMotor2;

        private double velocity;
        private final int P = 2;
        private final int ticks = 28;
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
                shooterMotor1.setPower(P*(Math.abs(velocity*2800-shooterMotor1.getVelocity())/(velocity*28)));
                shooterMotor2.setPower(P*(Math.abs(velocity*2800-shooterMotor2.getVelocity())/(velocity*28)));
                double vel = shooterMotor1.getVelocity();//2800 usually
                telemetryPacket.put("V1", vel);
            }


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
            this.finished = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(finished) {
                passerMotor.setPower(0.0);
                passerServo.setPower(0.0);
            } else {
                passerMotor.setPower(.75);
                passerServo.setPower(1.0);
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
}
