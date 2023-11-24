package org.firstinspires.ftc.teamcode.Roadrunner_Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season.Subsystems.TeamElementDetection.TeamElementSubsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Purple_Pixel extends LinearOpMode{
    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection=null;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }

    public void runOpMode() {

        HardwareStart();

        String curAlliance = "red";

        while (!opModeIsActive() && !isStopRequested()){
            element_zone = teamElementDetection.elementDetection(telemetry);

            teamElementDetection.setAlliance(curAlliance);

            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.addData("Zone", element_zone);
            telemetry.update();
        }
        telemetry.addData("Object", "Passed waitForStart");
        telemetry.update();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

        teamElementDetection.closePipeline();

        if (isStopRequested()) return;

        //Camera Detection and Purple Pixel
        //Code for Red Backboard or Blue Wing, other two reversed
        if (element_zone == 1) {
            telemetry.addLine("Zone 1 selected");
            telemetry.update();

            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                    .forward(25)
//                    .turn(Math.toRadians(90))
//                    .forward(5)
//                    .forward(-5)
//                    .turn(Math.toRadians(-90))
//                    .forward(-13)

                    .forward(10)

//                    .splineTo(new Vector2d(22,14), Math.toRadians(45))
//                    .splineTo(new Vector2d(0,0), Math.toRadians(0))

//                    .strafeLeft(5)
//                    .forward(5)
//                    .strafeLeft(5)
//                    .forward(5)
//                    .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(-30))

                    .turn(Math.toRadians(15))
                    .forward(5)
                    .turn(Math.toRadians(15))
                    .forward(10)

                    .forward(-10)
                    .turn(Math.toRadians(-30))
                    .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(0))

                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 2) {
            telemetry.addLine("Zone 2 selected");
            telemetry.update();

            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(30.5)
                    .waitSeconds(0.5)
                    .forward(-30.5)
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 3) {
            telemetry.addLine("Zone 3 selected");
            telemetry.update();

            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(22,-13), Math.toRadians(-15))
                    .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(0))
                    .build();
            drive.followTrajectorySequence(trajectory);
        }
    }
}