package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class AutonomousForDummies extends LinearOpMode {
    Orientation lastAngles = new Orientation();
    double                  correction;

    DriveTrain robot;
    Gyro gyro;

    private int option = 1;

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
            if(gamepad1.x || gamepad2.x) option = 1;
            if (gamepad1.b || gamepad2.b) option = 2;
            if(gamepad1.y) option = 3;
            if (gamepad1.a) option = 4;
            telemetry.addData("Option {Servo (x), Drive (B), Strafe (Y)", option);

            telemetry.update();
        }

        //wait for start button
        waitForStart();

        while (opModeIsActive()){
            correction = gyro.checkDirection();

            if(option == 1){
                robot.stoneClaw.setPosition(.3);
                sleep(30000);
            }
            else if (option == 2){
                robot.gyroDrive(.3, 12, 0);
                sleep(30000);
            }
            else if (option == 3){
                robot.gyroDrive(.3, 28, 0);
                robot.strafeRight(.3);
                sleep(1300);
                robot.stopMotors();
                sleep(30000);
            }
            else if (option == 4){
                robot.gyroDrive(.3, 28, 0);
                robot.strafeLeft(.3);
                sleep(1300);
                robot.stopMotors();
                sleep(30000);
            }
        }
    }
}
