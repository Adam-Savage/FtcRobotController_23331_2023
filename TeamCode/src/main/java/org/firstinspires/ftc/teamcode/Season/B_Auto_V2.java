package org.firstinspires.ftc.teamcode.Season;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season.Subsystems.TeamElementDetection.TeamElementSubsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class B_Auto_V2 extends LinearOpMode{
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

        if (isStopRequested()) return;

        //The Three Options
//        .splineTo(new Vector2d(30, 30), 0) - 0 is heading (relative)
//        .strafeRight(20)
//        .strafeLeft(20)
//        .forward(20)
//        .back(20)

        if (element_zone == 1) {

            Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(30, 30), 0)
                    .build();
            drive.followTrajectory(trajectory1);

            sleep(2000);

            drive.followTrajectory(
                    drive.trajectoryBuilder(trajectory1.end(), true)
                            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                            .build()
            );
        }

        else if (element_zone == 2) {
            Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(40)
                    .build();
            drive.followTrajectory(trajectory1);

            sleep(500);

            Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                    .back(30)
                    .build();
            drive.followTrajectory(trajectory2);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();

            while (!isStopRequested() && opModeIsActive()) ;
        }

        else if (element_zone == 3) {
            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                    .back(20)
                    .build();
            drive.followTrajectory(trajectory);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();

            while (!isStopRequested() && opModeIsActive()) ;
        }
    }
}
