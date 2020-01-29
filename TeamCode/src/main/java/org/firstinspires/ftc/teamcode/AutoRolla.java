package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

    @Override
    public void runOpMode(){
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
            telemetry.addData("2 global heading", gyro.globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            if(color == "Red") {
                robot.gyroDrive(DRIVE_SPEED, 32, 0);
                sleep(100);

                robot.gyroTurn(TURN_SPEED, -90);
                sleep(100);

                robot.gyroDrive(DRIVE_SPEED, -32, -90);
                sleep(100);

                robot.foundationClaw.setPosition(.2);
                sleep(750);

                robot.gyroDrive(DRIVE_SPEED, 48, -90);
                sleep(100);

                robot.foundationClaw.setPosition(.7);
                sleep(750);

                robot.strafeRight(1.0);
                sleep(1200);

                robot.stopMotors();
                sleep(30000);
            }
            if (color == "Blue")
            {
                robot.gyroDrive(DRIVE_SPEED,32,0 );
                sleep(100);

                robot.gyroTurn(TURN_SPEED,90);
                sleep(100);

                robot.gyroDrive(DRIVE_SPEED,-32,90);
                sleep(100);

                robot.foundationClaw.setPosition(.25);
                sleep(750);

                robot.gyroDrive(DRIVE_SPEED,48,90);
                sleep(100);

                robot.foundationClaw.setPosition(.7);
                sleep(750);

                robot.strafeLeft(1.0);
                sleep(1200);

                robot.stopMotors();
                sleep(30000);
            }
        }
    }
}
