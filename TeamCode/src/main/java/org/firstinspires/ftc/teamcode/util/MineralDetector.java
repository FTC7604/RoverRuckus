package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class MineralDetector {
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "Adw59PP/////AAABmSngvZTKXktpu+nuzpPLAFUc6w406s2RYiPPvJaY9A1k2/zyXeM83mHvqT14sWp9QlghcCK1akohLb6SHQv4cXvD8AbeO1a9sRhhchx1X5eL6ttrRE5PH6g517XhKI0dvKsoeYhZu6k4ln6dacQOC11xv/AHSEi/VipxqOMXlNesBfv/jmCc48H6LTFTOHLVDEb9vkk7btw6StRcwle0PUdbCh5aPIkRI2pTh+0R1hY5FyGGrdyZltrBoUusodgwQW0sIai/V21YZGgKaN5QYZLOhO3Fv0ZhjWsnj52e/BivDb3RJyPF1loygTBADo6YZoki1S/oDzoqcP3VmjIaEIFr6RfIGrnVZtkVbjWZP+Zs";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void mapHardware(OpMode opmode) {
        mapHardware(opmode.hardwareMap);
    }

    public void mapHardware(HardwareMap hardwareMap){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void sortArray(List<Recognition> arrayList){
        int size = arrayList.size();

        if(size >= 3) {
            for (int i = 1; i < size; i++) {
                for (int j = size - 1; j >= i; j--) {
                    if (arrayList.get(j - 1).getLeft() > arrayList.get(j).getLeft()) {
                        arrayList.set(j, arrayList.set(j - 1, arrayList.get(j)));
                    }
                }
            }
        }
    }

    public int detectMineral(){
        int mineralPosition = 0;
        int goldMineralX = -1;
        int silverMineral1X = -1;
        int silverMineral2X = -1;

        runtime.reset();

        while ((mineralPosition == 0 || runtime.seconds() < 2)) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                List<Recognition> validRecognitions = new ArrayList<>();

                if (updatedRecognitions != null) {

                    sortArray(updatedRecognitions);

                    if (updatedRecognitions.size() >= 3) {

                        validRecognitions.add(updatedRecognitions.get(0));
                        validRecognitions.add(updatedRecognitions.get(1));
                        validRecognitions.add(updatedRecognitions.get(2));

                        for (Recognition recognition : validRecognitions) {

                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getTop();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getTop();
                            } else {
                                silverMineral2X = (int) recognition.getTop();
                            }

                        }

                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {

                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                mineralPosition = 1;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                mineralPosition = 3;
                            } else {
                                mineralPosition = 2;
                            }

                        }

                    }
                }

            }
        }

        tfod.shutdown();

        return mineralPosition;
    }
}
