package org.firstinspires.ftc.teamcode.auton.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class ConeDetector {
    private OpenCvCamera camera;
    private final String cameraName;
    private PPConePipeline conePipeline;
    private final HardwareMap hardwareMap;
    private final int width, height;


    public ConeDetector(HardwareMap hMap, String camName) {
        cameraName = camName;
        hardwareMap = hMap;
        width = 432;
        height = 240;
    }

    public ConeDetector(HardwareMap hMap, String camName, int width, int height){
        cameraName = camName;
        hardwareMap = hMap;
        this.width = width;
        this.height = height;
    }

    public void init(){
        int cameraViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, cameraName), cameraViewId);

        conePipeline = new PPConePipeline();

        camera.setPipeline(conePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                RobotLog.addGlobalWarningMessage("Warning: Camera device failed to open with EasyOpenCv error: " +
                    ((errorCode == -1) ? "CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE" : "CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE"));
            }
        });
    }

    public OpenCvCamera getCamera(){
        return camera;
    }

    public int getColor(){
        return conePipeline.getColor();
    }
}
