package org.firstinspires.ftc.teamcode.Roadrunner_Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season.Subsystems.TeamElementDetection.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.Season.Z_Global_Variables;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (preselectTeleOp = "A_TeleOp_Nats")
public class B_Blue_Wing_Yellow extends LinearOpMode{

//---------------------------------------------------------------------------

    public int element_zone = 1;
    private TeamElementSubsystem teamElementDetection=null;

    int AutoLiftSetPt = Z_Global_Variables.AutoLiftSetPt;
    int LiftSetPtIntake = Z_Global_Variables.LiftSetPtIntake;
    double WristSetPtIn = Z_Global_Variables.WristSetPtIn;
    double WristSetPtScore = Z_Global_Variables.WristSetPtScore;
    double WristHoldPixel = Z_Global_Variables.WristHoldPixel;
    double ClawSetPtSingleSmall = Z_Global_Variables.ClawSetPtSingleSmall;
    double AutoClawSetPtOpen = Z_Global_Variables.AutoClawSetPtOpen;

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

            Wrist.setPosition(WristHoldPixel);

            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(30)
                    .turn(Math.toRadians(90))
                    .forward(2)
                    .build();
            drive.followTrajectorySequence(trajectory);

            Wrist.setPosition(WristSetPtIn);

            TrajectorySequence back = drive.trajectorySequenceBuilder(new Pose2d(30,-2,
                            Math.toRadians(90)))
                    .forward(-2)
                    .strafeLeft(25)
                    .build();
            drive.followTrajectorySequence(back);
        }

        else if (element_zone == 2) {
            telemetry.addLine("Zone 2 Detected");
            telemetry.update();

            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(30.5)
                    .waitSeconds(0.5)
                    .forward(-25.5)
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 3) {
            telemetry.addLine("Zone 3 Detected");
            telemetry.update();

            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(22,-11), Math.toRadians(-15))
                    .splineToConstantHeading(new Vector2d(5,0), Math.toRadians(0))
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

//---------------------------------------------------------------------------

        //Drive along Wall and Align Yellow
        telemetry.addLine("To Backboard");
        telemetry.update();

        if (element_zone == 1) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(5,4,
                            Math.toRadians(90)))
                    .turn(Math.toRadians(90))
                    .forward(72.5)
                    .strafeRight(19)
                    .forward(16)
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 2) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(5,0))
                    .turn(Math.toRadians(90))
                    .forward(72.5)
                    .strafeRight(27)
                    .forward(16)
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 3) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(5,0))
                    .turn(Math.toRadians(90))
                    .forward(72.5)
                    .strafeRight(34)
                    .forward(16)
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
        Wrist.setPosition(WristSetPtScore);
        Claw.setPosition(AutoClawSetPtOpen);

        //Lift Down
        Lift.setTargetPosition(LiftSetPtIntake);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(1);
        while (Lift.getCurrentPosition() != LiftSetPtIntake) {
            telemetry.addLine("Running to zero");
            telemetry.addData("Currently at", Lift.getCurrentPosition());
            telemetry.update();
        }
        Lift.setPower(0);

        //Retract wrist
        Wrist.setPosition(WristSetPtIn);

//---------------------------------------------------------------------------

        //Move back from backboard
        if (element_zone == 1) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(24, 88.5,
                            Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(24, 86, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 2) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(32, 88.5,
                            Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(32, 86, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(trajectory);
        }

        else if (element_zone == 3) {
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(40, 88.5,
                            Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(40, 86, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(trajectory);
        }
    }
}