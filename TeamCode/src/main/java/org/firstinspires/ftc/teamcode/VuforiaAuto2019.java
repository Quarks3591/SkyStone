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
@Disabled
public class VuforiaAuto2019 extends LinearOpMode {
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;
    float                   xValue;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.5;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.25;     // Nominal half speed for better accuracy.
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable

    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    Gyro gyro;
    DriveTrain robot;
    VuforiaLocalizer vuforia;

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
                correction = gyro.checkDirection();

                telemetry.addData("1 gyro heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();

                if (color == "Red") {

                }
                if (color == "Blue") {

                }
        }
    }
}
