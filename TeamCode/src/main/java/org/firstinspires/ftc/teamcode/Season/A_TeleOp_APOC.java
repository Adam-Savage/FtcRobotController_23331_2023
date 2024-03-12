package org.firstinspires.ftc.teamcode.Season;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
@TeleOp
public class A_TeleOp_APOC extends OpMode {

//---------------------------------------------------------------------------

    //Motor Set Points

    //Servo Set Points

    //Autonomous Function Sleeps

//---------------------------------------------------------------------------

    //Elevator PIDF Variables
    public PIDFController Elevator_controller;
    public static double E_p = 0, E_i = 0, E_d = 0;
    public static double E_f = 0;
    public static int Elevator_target = 0;

    //Intake PIDF Variables
    public PIDFController Intake_controller;
    public static double I_p = 0, I_i = 0, I_d = 0;
    public static double I_f = 0;
    public static int Intake_target = 0;

    //Climb PIDF Variables
    public PIDFController Climb_controller;
    public static double C_p = 0, C_i = 0, C_d = 0;
    public static double C_f = 0;
    public static int Climb_target = 0;

    //Wrist PIDF Variables
    public PIDFController Wrist_controller;
    public static double W_p = 0, W_i = 0, W_d = 0;
    public static double W_f = 0;
    public static int Wrist_target = 0;

//---------------------------------------------------------------------------

    //Limit Switch Definition
    DigitalChannel ElevatorLimit;
    DigitalChannel IntakeLimit;
    DigitalChannel ClimbHookLeft;
    DigitalChannel ClimbHookRight;
    DigitalChannel IntakePixel1;
    DigitalChannel IntakePixel2;

//---------------------------------------------------------------------------

    //Trigger state initialization
//    public double previousRTriggerState = 0;
//    public double previousLTriggerState = 0;

    //Lift Reset timer initialization
    public long lastResetTime = 0;

//---------------------------------------------------------------------------

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;


    public DcMotor Elevator;
    public DcMotor leftClimb;
    public DcMotor rightClimb;


    public Servo Wrist;
    public Servo openClaw;
    public Servo spinClaw;
    public Servo deployDrone;
    public Servo launchDrone;
    public Servo spinIntake;
    public Servo extendIntake;
    public Servo lockHandoff;
    public Servo leftClimbQuick;
    public Servo rightClimbQuick;


//---------------------------------------------------------------------------

    @Override
    public void init() {
        //Code to run ONCE when the driver hits INIT

//---------------------------------------------------------------------------

        //Motor Declaration
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Leftfront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Leftback");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Rightfront");
        backRightMotor = hardwareMap.get(DcMotor.class, "Rightback");

        //Motor Reverse
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//---------------------------------------------------------------------------

        //IMU Declaration
        IMU imu = hardwareMap.get(IMU.class, "imu");

        //Parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

//---------------------------------------------------------------------------

        //Motor Declaration
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        leftClimb = hardwareMap.get(DcMotor.class, "leftClimb");
        rightClimb = hardwareMap.get(DcMotor.class, "rightClimb");

        //Encoder Mode
        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Enable Break
        Elevator.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftClimb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightClimb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);


//---------------------------------------------------------------------------

        //PIDF Setup
        Elevator_controller = new PIDFController(E_p, E_i, E_d, E_f);
        Intake_controller = new PIDFController(I_p, I_i, I_d, I_f);
        Climb_controller = new PIDFController(C_p, C_i, C_d, C_f);
        Wrist_controller = new PIDFController(W_p, W_i, W_d, W_f);

        //Servo Setup
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        openClaw = hardwareMap.get(Servo.class, "openClaw");
        spinClaw = hardwareMap.get(Servo.class, "spinClaw");
        deployDrone = hardwareMap.get(Servo.class, "deployDrone");
        launchDrone = hardwareMap.get(Servo.class, "launchDrone");
        spinIntake = hardwareMap.get(Servo.class, "spinIntake");
        extendIntake = hardwareMap.get(Servo.class, "extendIntake");
        lockHandoff = hardwareMap.get(Servo.class, "lockHandoff");
        leftClimbQuick = hardwareMap.get(Servo.class, "leftClimbQuick");
        rightClimbQuick = hardwareMap.get(Servo.class, "rightClimbQuick");

        //Limit Switch Setup
        ElevatorLimit = hardwareMap.get(DigitalChannel.class, "ElevatorLimit");
        IntakeLimit = hardwareMap.get(DigitalChannel.class, "IntakeLimit");
        ClimbHookLeft = hardwareMap.get(DigitalChannel.class, "ClimbHookLeft");
        ClimbHookRight = hardwareMap.get(DigitalChannel.class, "ClimbHookRight");
        IntakePixel1 = hardwareMap.get(DigitalChannel.class, "IntakePixel1");
        IntakePixel2 = hardwareMap.get(DigitalChannel.class, "IntakePixel2");

//---------------------------------------------------------------------------

        //Initialise Servos
        Wrist.setPosition(0);
        openClaw.setPosition(0);
        spinClaw.setPosition(0);
        deployDrone.setPosition(0);
        launchDrone.setPosition(0);
        spinIntake.setPosition(0);
        extendIntake.setPosition(0);
        lockHandoff.setPosition(0);
        leftClimbQuick.setPosition(0);
        rightClimbQuick.setPosition(0);

        //Initialise PIDF Loops
        Elevator_target = 0;
        Intake_target = 0;
        Climb_target = 0;
        Wrist_target = 0;

//---------------------------------------------------------------------------

        //Send telemetry to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

//---------------------------------------------------------------------------

    }

//---------------------------------------------------------------------------

    @Override
    public void init_loop() {
        //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

//---------------------------------------------------------------------------

        //code here

//---------------------------------------------------------------------------

    }

//---------------------------------------------------------------------------

    @Override
    public void start() {
        //Code to run ONCE when the driver hits PLAY

//---------------------------------------------------------------------------

        //code here

//---------------------------------------------------------------------------

    }

//---------------------------------------------------------------------------

    @Override
    public void loop() {
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

//---------------------------------------------------------------------------
        //Variables

        //Driving Variables
        double LeftStickY;
        double LeftStickX;
        double RX;

//---------------------------------------------------------------------------

        //Drive Control

        //Slow Driving
        if (gamepad1.left_bumper) {
            LeftStickY = -gamepad1.left_stick_y * 0.25;
            LeftStickX = gamepad1.left_stick_x * 0.25;
            RX = gamepad1.right_stick_x * 0.3;
        }

        //Normal Driving
        else {
            LeftStickY = -gamepad1.left_stick_y;
            LeftStickX = gamepad1.left_stick_x;
            RX = gamepad1.right_stick_x * 0.8;
        }

//---------------------------------------------------------------------------

        //Field Centric Drive

        IMU imu = hardwareMap.get(IMU.class, "imu");

        //Reset IMU when START Pressed
        if (gamepad1.options) {
            imu.resetYaw();
        }

        //Get robot facing direction
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //Calculate rotation
        double rotX = LeftStickX * Math.cos(-botHeading) - LeftStickY * Math.sin(-botHeading);
        double rotY = LeftStickX * Math.sin(-botHeading) + LeftStickY * Math.cos(-botHeading);

        //Strafing Correction
        rotX = rotX * 1.2;

        //Maths for individual motor control
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(RX), 1);
        double frontLeftPower = (rotY + rotX + RX) / denominator;
        double backLeftPower = (rotY - rotX + RX) / denominator;
        double frontRightPower = (rotY - rotX - RX) / denominator;
        double backRightPower = (rotY + rotX - RX) / denominator;

        //Set Powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

//---------------------------------------------------------------------------

        //Update Variables

//---------------------------------------------------------------------------

        //Telemetry Update
        //Drive Information
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        //Update
        telemetry.update();

//---------------------------------------------------------------------------

    }

//---------------------------------------------------------------------------

    @Override
    public void stop() {
        //Code to run ONCE after the driver hits STOP

//---------------------------------------------------------------------------

        //code here

//---------------------------------------------------------------------------

    }

//---------------------------------------------------------------------------

//Functions Go Here

    private void sleep (int t) {
        try {
            Thread.sleep(t);
        } catch(InterruptedException e){
            throw new RuntimeException(e);
        }
    }

//---------------------------------------------------------------------------

}