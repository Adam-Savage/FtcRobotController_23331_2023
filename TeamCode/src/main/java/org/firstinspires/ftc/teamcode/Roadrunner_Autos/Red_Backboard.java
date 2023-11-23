package org.firstinspires.ftc.teamcode.Roadrunner_Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season.Subsystems.TeamElementDetection.TeamElementSubsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class Red_Backboard extends LinearOpMode{
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

        //Camera Detection and Purple Pixel
        if (element_zone == 1) {
            telemetry.addLine("Zone 1 Detected");
            telemetry.update();

            Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(10)
                    .build();
            drive.followTrajectory(trajectory1);
        }

        else if (element_zone == 2) {
            telemetry.addLine("Zone 2 Detected");
            telemetry.update();

            Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(10)
                    .build();
            drive.followTrajectory(trajectory1);
        }

        else if (element_zone == 3) {
            telemetry.addLine("Zone 3 Detected");
            telemetry.update();

            Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(10)
                    .build();
            drive.followTrajectory(trajectory1);
        }

        //Drive to Backboard
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(20, 30), 90)
                .build();
        drive.followTrajectory(trajectory1);

        sleep(500);

        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 50), 0)
                .build();
        drive.followTrajectory(trajectory2);
    }
}