package org.firstinspires.ftc.teamcode.Roadrunner_Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season.Subsystems.TeamElementDetection.TeamElementSubsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class A_Red_Wing_Crossfield extends LinearOpMode{
    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection=null;

    public static double WristSetPtIn = 0.38;
    public static double WristSetPtScore = 0.44;
    public static double ClawSetPtSingleSmall = 1;
    public static double ClawSetPtOpen = 0.88;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        //Servo Declaration
        Servo Wrist = hardwareMap.servo.get("Wrist");
        Servo Claw = hardwareMap.servo.get("Claw");

        //Initialise Servos
        Claw.setPosition(ClawSetPtSingleSmall);
        Wrist.setPosition(WristSetPtIn);

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

        //Servo Declaration
        Servo Wrist = hardwareMap.servo.get("Wrist");
        Servo Claw = hardwareMap.servo.get("Claw");

        waitForStart();

        teamElementDetection.closePipeline();

        if (isStopRequested()) return;

        //Camera Detection and Purple Pixel
        if (element_zone == 1) {
            telemetry.addLine("Zone 1 Detected");
            telemetry.update();

            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(22,11), Math.toRadians(-15))
                    .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(0))
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 2) {
            telemetry.addLine("Zone 2 Detected");
            telemetry.update();

            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(30.5)
                    .waitSeconds(0.5)
                    .forward(-30.5)
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 3) {
            telemetry.addLine("Zone 3 Detected");
            telemetry.update();

            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(14)

                    .turn(Math.toRadians(-15))
                    .forward(5)
                    .turn(Math.toRadians(-15))
                    .forward(10)

                    .forward(-10)
                    .turn(Math.toRadians(30))
                    .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(0))
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        //Wait to drive along wall
        sleep(100);

        //Drive along Wall to Park
        telemetry.addLine("To Park");
        telemetry.update();

        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(5)
                .turn(Math.toRadians(-90))
                .forward(90)
                .build();
        drive.followTrajectorySequence(trajectory);

        //Put down Yellow
        Wrist.setPosition(WristSetPtScore);
        sleep(500);
        Claw.setPosition(ClawSetPtOpen);
        sleep(500);
    }
}