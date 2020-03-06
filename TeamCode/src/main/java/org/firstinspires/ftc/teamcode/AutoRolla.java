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

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.3;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.15;     // Nominal half speed for better accuracy.

    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable

    //init input variables
    private String color = "Red";
    private String side = "Left";

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

            if(gamepad1.a || gamepad2.a) side = "Left";
            if (gamepad1.y || gamepad2.y) side = "Right";
            telemetry.addData("Side {Left (A), Right (Y)}", side);

            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);

            telemetry.update();
        }

        //wait for start button
        waitForStart();
        runtime.reset();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        //loop until end of autonomous period
        while (opModeIsActive()) {
            correction = gyro.checkDirection();

            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);

            if (valLeft == 0 && valMid > 0 && valRight > 0){
                robot.gyroDrive(1.0, 35, 0);
                sleep(250);

                robot.stoneClaw.setPosition(0.1);
                sleep(250);

                //lift

                robot.gyroDrive(1.0, -18, 0);
                sleep(250);

                robot.gyroTurn(0.3, -90);
                sleep(250);

                robot.gyroDrive(.7, 78, -90);
                sleep(250);

                robot.gyroTurn(0.3, -180);
                sleep(250);

                robot.gyroDrive(1.0, -10, -180);
                sleep(250);

                while(robot.digitalTouch.getState() == true){
                    robot.drive(-.1);
                }
                robot.stopMotors();
                sleep(250);

                //FOUNDATION CLAW POSITION

                robot.gyroTurn(.3, -90);
            }
            if (valMid == 0 && valLeft > 0 && valRight > 0){
                robot.strafeRight(1.0);
                sleep(150); //replace this with a strafe using encoders if possible
                robot.stopMotors();
                sleep(250);

                robot.gyroDrive(.7, 35, 0);
                sleep(250);

                robot.stoneClaw.setPosition(0.1);
                sleep(1000);

                //lift

                robot.gyroDrive(1.0, -18, 0);
                sleep(250);

                robot.gyroTurn(0.3, -90);
                sleep(250);

                robot.gyroDrive(.7, 78, -90);
                sleep(250);

                robot.gyroTurn(0.3, -180);
                sleep(250);

                robot.gyroDrive(1.0, -10, -180);
                sleep(250);

                while(robot.digitalTouch.getState() == true){
                    robot.drive(-.1);
                }
                robot.stopMotors();
                sleep(250);

                //FOUNDATION CLAW POSITION

                robot.gyroTurn(.3, -90);

            }
            if (valRight == 0 && valLeft > 0 && valMid > 0){
                robot.strafeRight(1.0);
                sleep(300); //replace this with a strafe using encoders if possible
                robot.stopMotors();
                sleep(250);

                robot.gyroDrive(1.0, 35, 0);
                sleep(250);

                robot.stoneClaw.setPosition(0.1);
                sleep(250);

                //lift

                robot.gyroDrive(1.0, -18, 0);
                sleep(250);

                robot.gyroTurn(0.3, -90);
                sleep(250);

                robot.gyroDrive(.7, 78, -90);
                sleep(250);

                robot.gyroTurn(0.3, -180);
                sleep(250);

                robot.gyroDrive(1.0, -10, -180);
                sleep(250);

                while(robot.digitalTouch.getState() == true){
                    robot.drive(-.1);
                }
                robot.stopMotors();
                sleep(250);

                //FOUNDATION CLAW POSITION

                robot.gyroTurn(.3, -90);
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
