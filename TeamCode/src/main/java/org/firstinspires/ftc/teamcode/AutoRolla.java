package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutoRolla extends LinearOpMode {
    Orientation             lastAngles = new Orientation();
    double                  correction;

    Gyro gyro;
    DriveTrain robot;

    //init input variables
    private String color = "Blue";
    private int blocks = 2;
    int cm = 1; //Color Modifier
    int sm = 0; //Skystone Modifier

    public static int BLOCK_WIDTH = 8; //in inches
    public static double DRIVE_SPEED = .7;

    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0.5f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPSIDE_DOWN);//display on RC

        gyro = new Gyro(this);
        robot = new DriveTrain(this, gyro);

        robot.init(hardwareMap);
        gyro.init(hardwareMap);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        gyro.checkCalibration();

        //side/color input for match
        while (!opModeIsActive()) {
            if(gamepad1.x || gamepad2.x) color = "Blue";
            if (gamepad1.b || gamepad2.b) color = "Red";
            telemetry.addData("Color {Blue (X), Red (B)}", color);

            if(gamepad1.a || gamepad2.a) blocks--;
            if (gamepad1.y || gamepad2.y) blocks++;
            telemetry.addData("Blocks { - (A), + (Y)}", blocks);

            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);

            telemetry.update();
        }

        //wait for start button
        waitForStart();
        runtime.reset();

        sleep(1000);

        if (color == "Red")
            cm = -1;
        if (valMid == 0 && valLeft > 0 && valRight > 0)
            sm = 1;
        if (valRight == 0 && valLeft > 0 && valMid > 0)
            sm = 2;


        //loop until end of autonomous period
        while (opModeIsActive()) {
            correction = gyro.checkDirection();

            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);

            if (valLeft == 0 && valMid > 0 && valRight > 0){
                robot.strafeRight(1.0);
                sleep(150*sm);
                robot.stopMotors();
                sleep(250);

                robot.gyroDrive(DRIVE_SPEED, 35, 0);
                sleep(250);

                robot.stoneClaw.setPosition(0.1);
                sleep(250);

                robot.gyroDrive(DRIVE_SPEED, -18, 0);
                sleep(250);

                robot.gyroTurn(0.3, -90*cm);
                sleep(250);

                robot.gyroDrive(DRIVE_SPEED, 86 - sm*BLOCK_WIDTH, -90*cm);
                sleep(250);

                robot.gyroTurn(0.3, -180*cm);
                sleep(250);

                robot.gyroDrive(DRIVE_SPEED, -10, -180*cm);
                sleep(250);

                while(robot.digitalTouch.getState() == true){
                    robot.drive(-.1);
                }
                robot.stopMotors();
                sleep(250);

                robot.foundationClawRight.setPosition(0.7);
                robot.foundationClawLeft.setPosition(0.2);
                sleep(1000);

                robot.gyroTurn(.3, -90*cm);

                robot.gyroDrive(DRIVE_SPEED/2, -12, -90*cm);

                robot.foundationClawRight.setPosition(0.2);
                robot.foundationClawLeft.setPosition(0.7);
                sleep(1000);

                robot.gyroDrive(DRIVE_SPEED, 9, -90*cm);

                robot.gyroTurn(.3, 90*cm);

                //lift

                robot.gyroDrive(DRIVE_SPEED, 9, 90*cm);

                robot.stoneClaw.setPosition(.5);

                robot.strafeLeft(DRIVE_SPEED);
                sleep(500);
                robot.stopMotors();
                sleep(250);
                if(blocks >= 2) {
                    robot.gyroDrive(DRIVE_SPEED, 122 - sm * BLOCK_WIDTH, -90 * cm);

                    robot.gyroTurn(.3, -90*cm);

                    robot.gyroDrive(DRIVE_SPEED, 35, 0);
                    sleep(250);

                    robot.stoneClaw.setPosition(0.1);
                    sleep(250);

                    robot.gyroDrive(DRIVE_SPEED, -18, 0);
                    sleep(250);

                    robot.gyroTurn(0.3, -90*cm);
                    sleep(250);

                    robot.gyroDrive(DRIVE_SPEED, 122 - sm*BLOCK_WIDTH, -90*cm);
                    sleep(250);

                }
            }
        }
    }
    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private OpenCV3939.StageSwitchingPipeline.Stage stageToRenderToViewport = OpenCV3939.StageSwitchingPipeline.Stage.detection;
        private OpenCV3939.StageSwitchingPipeline.Stage[] stages = OpenCV3939.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                        return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}
