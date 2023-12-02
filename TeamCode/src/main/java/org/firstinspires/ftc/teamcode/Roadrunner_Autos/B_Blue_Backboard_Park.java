package org.firstinspires.ftc.teamcode.Roadrunner_Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season.Subsystems.TeamElementDetection.TeamElementSubsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (preselectTeleOp = "A_TeleOp_Syd")
public class B_Blue_Backboard_Park extends LinearOpMode{

//---------------------------------------------------------------------------

    public int element_zone = 1;
    private TeamElementSubsystem teamElementDetection=null;

    public static int AutoLiftSetPt = 300;
    public static double WristSetPtIn = 0.38;
    public static double WristSetPtScore = 0.44;
    public static double ClawSetPtSingleSmall = 1;
    public static double ClawSetPtOpen = 0.88;
    public static double AutoClawSetPtOpen = 0.7;

//---------------------------------------------------------------------------

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

//---------------------------------------------------------------------------

    public void runOpMode() {

        HardwareStart();

//---------------------------------------------------------------------------

        String curAlliance = "blue";

        while (!opModeIsActive() && !isStopRequested()){
            element_zone = teamElementDetection.elementDetection(telemetry);

            teamElementDetection.setAlliance(curAlliance);

            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.addData("Zone", element_zone);

            telemetry.update();

        }
        telemetry.addData("Object", "Passed waitForStart");
        telemetry.update();

//---------------------------------------------------------------------------

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Servo Declaration
        Servo Wrist = hardwareMap.servo.get("Wrist");
        Servo Claw = hardwareMap.servo.get("Claw");

        //Lift Things
        DcMotor Lift = hardwareMap.dcMotor.get("Lift");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//---------------------------------------------------------------------------

        waitForStart();

        teamElementDetection.closePipeline();

        if (isStopRequested()) return;

//---------------------------------------------------------------------------

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
            telemetry.addLine("Zone 3 selected");
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

//---------------------------------------------------------------------------

        //Drive to Backboard
        telemetry.addLine("To Backboard");
        telemetry.update();

        if (element_zone == 1) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .splineTo(new Vector2d(18, 35), Math.toRadians(90))
                    .strafeRight(5)
                    .forward(6)
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 2) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .splineTo(new Vector2d(18, 35), Math.toRadians(90))
                    .strafeRight(15)
                    .forward(6)
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 3) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .splineTo(new Vector2d(18, 35), Math.toRadians(90))
                    .strafeRight(20)
                    .forward(6)
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

//---------------------------------------------------------------------------

        //Score Yellow
        //Lift Up
        Lift.setTargetPosition(AutoLiftSetPt);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(1);
        while (Lift.getCurrentPosition() != AutoLiftSetPt) {
            telemetry.addLine("Running to Set pt");
            telemetry.addData("Currently at", Lift.getCurrentPosition());
            telemetry.update();
        }
        Lift.setPower(0);

        //Claw Things
        sleep(500);
        Wrist.setPosition(WristSetPtScore);
        sleep(500);
        Claw.setPosition(AutoClawSetPtOpen);

        //Lift Down
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(1);
        while (Lift.getCurrentPosition() != 0) {
            telemetry.addLine("Running to zero");
            telemetry.addData("Currently at", Lift.getCurrentPosition());
            telemetry.update();
        }
        Lift.setPower(0);

//---------------------------------------------------------------------------

        //Move back from backboard
        if (element_zone == 1) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(23, 35, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(50, 33, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 2) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(33, 35, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(50, 33, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 3) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(38, 35, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(50, 33, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(trajectory);
        }
    }
}