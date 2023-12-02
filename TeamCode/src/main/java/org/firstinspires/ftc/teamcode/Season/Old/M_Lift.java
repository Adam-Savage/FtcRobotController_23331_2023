package org.firstinspires.ftc.teamcode.Season.Old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Disabled
@TeleOp
public class M_Lift extends LinearOpMode {

//---------------------------------------------------------------------------

//    //Set Speed
//    static final double ManualLiftSpeed = -0.5;
//    static final double AutoLiftSpeed = -1;

//---------------------------------------------------------------------------

//    //Set Endpoints
//    int maxLiftEncoderCount = 5000;
//    int minLiftEncoderCount = 0;

//---------------------------------------------------------------------------

//    //Set Set points
//    int LiftSetPtIntake = 100;
//    int LiftSetPtLvl1 = 2000;
//    int LiftSetPtLvl2 = 4000;

    //Servo Set Points
    static final double WristSetPtIn = 0.6;
    static final double WristSetPtOut = 0.21;
    static final double WristSetPtScore = 0.45;

    static final double ClawSetPtClosed = 0.02;
    static final double ClawSetPtOpen = 0.11;
    static final double ClawSetPtSingleSmall = 0;
    static final double ClawSetPtSingleWide = 0.05;

    //Motor Set Points
    int LiftSetPtIntake = 0;
    int LiftSetPtLvl1 = 600;
    int LiftSetPtLvl2 = 1500;

    int ClimbSetPtUp = -3000;
    int ClimbSetPtDown = -10;

//---------------------------------------------------------------------------

    private PIDController controller;

    public static double p = 0.04, i = 0.001, d = 0.0001;
    public static double f = 0.0001;

    public static int target = 0;

    private DcMotorEx Lift;

    @Override
    public void runOpMode() throws InterruptedException {

//---------------------------------------------------------------------------

        //Motor Declaration
        DcMotor Lift = hardwareMap.dcMotor.get("Lift");

//---------------------------------------------------------------------------

//        //Encoder Mode
//        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        //Enable Break
//        Lift.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
//
//        //Verify Robot Waiting
//        telemetry.addData(">", "Robot Ready.  Press Play.");
//        telemetry.update();

        //Servo Declaration
        Servo Wrist = hardwareMap.servo.get("Wrist");
        Servo Claw = hardwareMap.servo.get("Claw");

        //Initialise Servos
        Claw.setPosition(ClawSetPtClosed);
        Wrist.setPosition(WristSetPtIn);

//---------------------------------------------------------------------------

            controller = new PIDController(p, i, d);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

//            //Initialise Encoders
//            int currentLiftPosition = Lift.getCurrentPosition();
//
//            //Lift Soft Limits
//            if (currentLiftPosition < minLiftEncoderCount) {
//                Lift.setTargetPosition(minLiftEncoderCount+10);
//                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Lift.setPower(1);
//                while (opModeIsActive() && Lift.isBusy()) {
//                    currentLiftPosition = Lift.getCurrentPosition();
//                    if(Math.abs(Lift.getTargetPosition() - currentLiftPosition) > 50) {
//                        break;
//                    }
//                }
//                Lift.setPower(LiftSpeed);
//            } else if (currentLiftPosition > maxLiftEncoderCount) {
//                Lift.setPower(0.0);
//                Lift.setTargetPosition(maxLiftEncoderCount);
//                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }  else {
//                Lift.setPower(LiftSpeed);
//            }
//
//            //GPT Test Set Point
//            if (gamepad1.a) {
//                // 'A' button is pressed, set the target position and power
//                Lift.setTargetPosition(50);
//                Lift.setPower(1);
//
//                // Set the motor run mode to RUN_TO_POSITION
//                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                // Wait until the motor reaches the target position
//                while (opModeIsActive() && Lift.isBusy()) {
//                    // Optionally, you can add telemetry to display motor position
//                    telemetry.addData("Motor Position", Lift.getCurrentPosition());
//                    telemetry.update();
//                }
//
//                // Stop the motor when it reaches the target position
//                Lift.setPower(0.0);
//
//                // Set the motor run mode back to RUN_USING_ENCODER
//                Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            } else if (gamepad1.b) {
//                Lift.setTargetPosition(900);
//                Lift.setPower(1);
//
//                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                while (opModeIsActive() && Lift.isBusy()) {
//                    telemetry.addData("Motor Position", Lift.getCurrentPosition());
//                    telemetry.update();
//                }
//
//                Lift.setPower(0);
//                Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            } else {
//                double lifting = gamepad1.right_stick_y;
//                Lift.setPower(lifting * ManualLiftSpeed);
//            }

            //dad's soft limits
//            else {
//                // No button (a,b,y) is pressed, so see if the joystick is moved, but keep lift within min <> max
//                if (Lift.getCurrentPosition() < maxLiftEncoderCount && Lift.getCurrentPosition() > minLiftEncoderCount) {
//                    // Lift is in a safe place (between max and min) so action what the joystick says
//                    Lift.setPower(gamepad1.right_stick_y * ManualLiftSpeed);
//                } else if (Lift.getCurrentPosition() >= maxLiftEncoderCount) {
//                    // Lift is above max, so bounce down a little
//                    Lift.setPower(LiftBounceDown * AutoLiftSpeed);
//                } else if (Lift.getCurrentPosition() <= minLiftEncoderCount){
//                    // Lift is below min so bounce up a little
//                    Lift.setPower(LiftBounceUp * AutoLiftSpeed);
//                } // end if lift in safe zone
//            } // end if button pressed



            if (gamepad1.a) {
                Claw.setPosition(ClawSetPtClosed);
                Wrist.setPosition(WristSetPtIn);
                target = 0;
            }

            else if (gamepad1.x) {
                Claw.setPosition(ClawSetPtOpen);
                Wrist.setPosition(WristSetPtOut);
                target = LiftSetPtIntake;
            }

            else if (gamepad1.b) {
                Wrist.setPosition(WristSetPtScore);
                target = LiftSetPtLvl1;
            }

            else if (gamepad1.y) {
                Wrist.setPosition(WristSetPtScore);
                target = LiftSetPtLvl2;
            }

            controller.setPID(p, i, d);
            int LiftPos = Lift.getCurrentPosition();
            double pid = controller.calculate(LiftPos, target);
            double ff = target * f;

            double power = pid + ff;

            Lift.setPower(power);

            telemetry.addData("pos", LiftPos);
            telemetry.addData("target", target);
            telemetry.update();



//            //Telemetry Update
//            telemetry.addData("Lift Power", gamepad1.right_stick_y);
//            telemetry.addData("Lift Position", Lift.getCurrentPosition());
//            telemetry.update();

            }
        }
    }
