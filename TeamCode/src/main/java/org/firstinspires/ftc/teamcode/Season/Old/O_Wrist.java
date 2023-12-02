package org.firstinspires.ftc.teamcode.Season.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class O_Wrist extends LinearOpMode {

    boolean WristOut = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //Servo Declaration
        Servo Wrist = hardwareMap.servo.get("Wrist");

        //Initialise Servo
        Wrist.setPosition(0.0);

        // Track the previous state of the button
        boolean previousLBumperButtonState = false;

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Check the current state of buttons
            boolean currentLBumperButtonState = gamepad1.left_bumper;

            //Toggle Grab
            // Check if the button is currently pressed and was not pressed in the previous iteration
            if (currentLBumperButtonState && !previousLBumperButtonState) {
                if (WristOut) {
                    Wrist.setPosition(0.05);
                    // Wrist in
                } else {
                    Wrist.setPosition(0.3);
                    // Wrist out
                }
                WristOut = !WristOut; // Toggle the flag
            }

            previousLBumperButtonState = currentLBumperButtonState;
            // Update the previous button state

            //HOLD TO POSITION - Wrist Control
//            if (gamepad2.dpad_up)
//                Wrist.setPosition(-1);
//            else if (gamepad2.dpad_down)
//                Wrist.setPosition(0.4);
//            else
//                Wrist.setPosition(0);

            //Telemetry Update
            telemetry.addData("Wrist State", WristOut ? "Out" : "In");
            telemetry.update();
        }
    }
}
