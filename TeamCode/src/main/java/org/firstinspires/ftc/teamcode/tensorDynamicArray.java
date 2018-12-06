package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "Tensor Test & Dynamic List", group = "Linear Op")
public class tensorDynamicArray extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private static int numLoops = 0;
    private static int size = 0;

    @Override
    public void runOpMode() {
        int mineralPosition = 0;
        String mineralPositionString;
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        waitForStart();
        runtime.reset();

        if (tfod != null) {
            tfod.activate();
        }

        while (opModeIsActive() && mineralPosition == 0 || runtime.seconds() < 3) {
            mineralPosition = flowTensor();
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        telemetry.clearAll();
        switch(mineralPosition){
            case 1:
                mineralPositionString = "Left";
                break;
            case 2:
                mineralPositionString = "Center";
                break;
            case 3:
                mineralPositionString = "Right";
                break;
            default:
                mineralPositionString = "Error while reading";
                break;
        }
        telemetry.addData("Mineral position", mineralPositionString);
        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("Loops", numLoops);
        telemetry.update();

        sleep(3000);
    }

    //Dirty Vuforia things
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "Adw59PP/////AAABmSngvZTKXktpu+nuzpPLAFUc6w406s2RYiPPvJaY9A1k2/zyXeM83mHvqT14sWp9QlghcCK1akohLb6SHQv4cXvD8AbeO1a9sRhhchx1X5eL6ttrRE5PH6g517XhKI0dvKsoeYhZu6k4ln6dacQOC11xv/AHSEi/VipxqOMXlNesBfv/jmCc48H6LTFTOHLVDEb9vkk7btw6StRcwle0PUdbCh5aPIkRI2pTh+0R1hY5FyGGrdyZltrBoUusodgwQW0sIai/V21YZGgKaN5QYZLOhO3Fv0ZhjWsnj52e/BivDb3RJyPF1loygTBADo6YZoki1S/oDzoqcP3VmjIaEIFr6RfIGrnVZtkVbjWZP+Zs";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void sortArray(List<Recognition> arrayList){
        int size = arrayList.size();

        for(int i = 1; i < size; i++){
            for(int j = size - 1; j >= i; j--){
                if(arrayList.get(j - 1).getTop() < arrayList.get(j).getTop()){
                    arrayList.set(j, arrayList.set(j - 1, arrayList.get(j)));
                }
            }
        }
    }

    private int flowTensor(){
        int mineralPosition = 0;
        int goldMineralX = -1;
        int silverMineral1X = -1;
        int silverMineral2X = -1;

        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            List<Recognition> silverMinerals = new ArrayList<>();
            List<Recognition> goldMinerals = new ArrayList<>();

            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                for(Recognition recognition : updatedRecognitions){
                    if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMinerals.add(recognition);
                        telemetry.addLine("Added valid recognition " + recognition.getLabel());
                        telemetry.addData(recognition.getLabel() + " Y Value", (int) recognition.getTop());
                    } else{
                        silverMinerals.add(recognition);
                        telemetry.addLine("Added valid recognition " + recognition.getLabel());
                        telemetry.addData(recognition.getLabel() + " Y Value", (int) recognition.getTop());
                    }
                }

                sortArray(silverMinerals);
                sortArray(goldMinerals);

                for(int i = 2; i < silverMinerals.size(); i++){
                    silverMinerals.remove(2);
                }

                for(int i = 1; i < goldMinerals.size(); i++){
                    goldMinerals.remove(1);
                }

                updatedRecognitions.clear();
                updatedRecognitions.add(goldMinerals.get(0));
                updatedRecognitions.add(silverMinerals.get(1));
                updatedRecognitions.add(silverMinerals.get(2));

                if (updatedRecognitions.size() == 3) {

                    for (Recognition recognition : updatedRecognitions) {

                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                            telemetry.addData("Gold Mineral Y Value", (int) recognition.getTop());
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                            telemetry.addData("Silver Mineral Y Value", (int) recognition.getTop());
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                            telemetry.addData("Silver Mineral Y Value", (int) recognition.getTop());
                        }

                    }

                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {

                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            mineralPosition = 1;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            mineralPosition = 3;
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            mineralPosition = 2;
                        }

                    }

                }

                telemetry.update();
            }

        }

        numLoops++;
        telemetry.clearAll();
        return mineralPosition;
    }
}