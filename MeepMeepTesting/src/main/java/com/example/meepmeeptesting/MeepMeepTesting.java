package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-53.0, -53.0, Math.toRadians(45)))


                .strafeToLinearHeading(new Vector2d(-18.0, -15.0), Math.toRadians(43.0))
                // start to shoot




                // move to the first row
                .strafeToLinearHeading(new Vector2d(-11.0, -16.0), Math.toRadians(90.0))
                // move down
                .strafeToLinearHeading(new Vector2d(-11.0, -50.0), Math.toRadians(90.0)) // y was originally 49. im testing it as 40
                // move up
                .strafeToLinearHeading(new Vector2d(-11.0, -22.0), Math.toRadians(90.0))

                // Move to shoot
                .strafeToLinearHeading(new Vector2d(-18.0, -15.0), Math.toRadians(43.0))


                // Move to the second row
                .strafeToLinearHeading(new Vector2d(11.0, -22.0), Math.toRadians(90.0))
                // move down
                .strafeToLinearHeading(new Vector2d(11.0, -50.0), Math.toRadians(90.0))
                // move up
                .strafeToLinearHeading(new Vector2d(11.0, -22.0), Math.toRadians(90.0))

                // Move to shoot
                .strafeToLinearHeading(new Vector2d(-18.0, -15.0), Math.toRadians(43.0))


                // move to the third row
                .strafeToLinearHeading(new Vector2d(35.0, -18.0), Math.toRadians(90.0))
                // move down
                .strafeToLinearHeading(new Vector2d(34.0, -52), Math.toRadians(90.0))
                // move up
                .strafeToLinearHeading(new Vector2d(34.0, -22.0), Math.toRadians(90.0))

                // Move to shoot
                .strafeToLinearHeading(new Vector2d(-18.0, -15.0), Math.toRadians(43.0))

                .build());





        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}