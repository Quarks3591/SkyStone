package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous
public class TensorFlowAuto2019 extends LinearOpMode {
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;
    float                   xValue;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.5;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.25;     // Nominal half speed for better accuracy.
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable

    Gyro gyro;
    DriveTrain robot;

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    //init input variables
    private String color = "Red";
    private String side = "Left";
    private boolean foundation = true;

    @Override
    public void runOpMode(){
        gyro = new Gyro(this);
        robot = new DriveTrain(this, gyro);

        robot.init(hardwareMap);
        gyro.init(hardwareMap);

        robot.initVuforia();
        if(ClassFactory.getInstance().canCreateTFObjectDetector())
            robot.initTfod();
        else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        gyro.checkCalibration();

        //side/color input for match
        while (!opModeIsActive()) {
            if (gamepad1.x) color = "Blue";
            if (gamepad1.b) color = "Red";
            telemetry.addData("Color {Blue (X), Red (B)}", color);

            if (gamepad1.dpad_left) side = "Left";
            if (gamepad1.dpad_right) side = "Right";
            telemetry.addData("Side {Left (A), Right (Y)}", side);

            if (gamepad1.a) foundation = true;
            if (gamepad1.y) foundation = false;
            telemetry.addData("Foundation {Yes (A), No (Y)", foundation);

            telemetry.update();
        }

        //wait for start button
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        //loop until end of autonomous period
        while (opModeIsActive()) {
            if (tfod != null) {
                correction = gyro.checkDirection();

                telemetry.addData("1 gyro heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (color == "Red") {
                    while(!updatedRecognitions.contains("SkyStone")){
                        robot.strafeLeft(.5);
                    }
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == "SkyBlock") {
                            xValue = (recognition.getRight() + recognition.getLeft())/2;
                            telemetry.addData(String.format("Xvalue (%d)", i), xValue);
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            break;
                        }
                    }
                    telemetry.update();
                    if(xValue < 0){
                        robot.strafeLeft(.1);
                        sleep(50* Math.abs((int) xValue));
                        robot.stopMotors();
                        sleep(100);
                        robot.gyroDrive(DRIVE_SPEED, 24, 0);
                        sleep(100);
                        robot.stoneClaw.setPosition(0);
                        sleep(100);
                        robot.lift.setPower(.4);
                        sleep(400);
                        robot.lift.setPower(0);
                        sleep(100);
                        robot.gyroDrive(DRIVE_SPEED, -24, 0);
                        sleep(100);
                        gyro.getError(-90);
                        gyro.getSteer(gyro.robotError, P_TURN_COEFF);
                        robot.gyroTurn(TURN_SPEED,-90);
                        sleep(100);
                        robot.gyroDrive(DRIVE_SPEED, 60, -90);
                        sleep(100);

                    }
                    if(xValue > 0){
                        robot.strafeRight(.1);
                        sleep(50* Math.abs((int) xValue));
                        robot.stopMotors();
                        sleep(100);
                        robot.gyroDrive(DRIVE_SPEED, 24, 0);
                    }

                }
                if (color == "Blue") {

                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }

    }



}
