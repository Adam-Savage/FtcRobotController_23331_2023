package org.firstinspires.ftc.teamcode.Season;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Season.Pipeline.SplitAveragePipeline;

import java.util.Locale;

@Config
@Disabled
@TeleOp
public class EasyOpenCVTest extends LinearOpMode {
    OpenCvCamera camera;
    SplitAveragePipeline splitAveragePipeline;
    int camW = 800;
    int camH = 448;

    int zone = 1;

    public int element_zone = 1;

    boolean togglePreview = true;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "c270"));
        splitAveragePipeline = new SplitAveragePipeline();

        camera.setPipeline(splitAveragePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        String curAlliance = "red";

        while (!opModeIsActive() && !isStopRequested()){
            element_zone = splitAveragePipeline.get_element_zone();
            if (togglePreview && gamepad2.a){
                togglePreview = false;
                splitAveragePipeline.toggleAverageZonePipe();
            }else if (!gamepad2.a){
                togglePreview = true;
            }


            if (gamepad1.x){
                curAlliance = "blue";
            }else if (gamepad1.b){
                curAlliance = "red";
            }
            splitAveragePipeline.setAlliancePipe(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());


            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();
    }
}
