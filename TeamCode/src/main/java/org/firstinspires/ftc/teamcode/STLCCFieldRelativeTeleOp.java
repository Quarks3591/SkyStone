package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class STLCCFieldRelativeTeleOp extends OpMode
{
    //Wheels
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;

    DcMotor m0 = null;
    DcMotor m1 = null;
    DcMotor m2 = null;
    DcMotor m3 = null;

    //Extra Motors/Servos
    DcMotor lift;
    Servo stoneClaw;
    Servo foundationClaw;
    BNO055IMU gyro;

    double reset_angle = 0;

    int clawToggle = 0;
    int foundationToggle = 0;
    int slowmoToggle = 0;

    boolean slowmo = false;

    public void init()
    {
        //Wheel Initiation
        frontRightMotor = hardwareMap.dcMotor.get("front_right"); m0 = frontRightMotor;
        backRightMotor = hardwareMap.dcMotor.get("back_right"); m1 = backRightMotor;
        frontLeftMotor = hardwareMap.dcMotor.get("front_left"); m3 = frontLeftMotor;
        backLeftMotor = hardwareMap.dcMotor.get("back_left"); m2 = backLeftMotor;

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Extra Initiation
        lift = hardwareMap.dcMotor.get("lift");
        stoneClaw = hardwareMap.servo.get("claw");
        foundationClaw = hardwareMap.servo.get("foundation");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        gyro.initialize(parameters);

        stoneClaw.scaleRange(0, 1);

        while (gyro.isGyroCalibrated()) { }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("gyro calib status", gyro.getCalibrationStatus().toString());
        telemetry.update();
    }
    public void loop()
    {
        drive();
        resetAngle();
        setSlowmoToggle();
        stoneClawControl();
        foundationClawControl();
        liftControl();
    }
    public void drive() {
        double drivescale = 1.5;
        double rotatescale = 2;
        double Protate = -gamepad1.right_stick_x/rotatescale;
        double stick_x = -gamepad1.left_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/drivescale); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = gamepad1.left_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/drivescale);
        double theta = 0;
        double Px = 0;
        double Py = 0;

        double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians
        if (gyroAngle <= 0) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        //gyroAngle = -1 * gyroAngle;

        if(gamepad1.left_bumper){ //Disables gyro, sets to -Math.PI/2 so front is defined correctly.
            gyroAngle = -Math.PI/2;
        }

        //MOVEMENT
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));

        telemetry.addData("Gyro", getHeading());

        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("Magnitude",  Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
        telemetry.addData("Front Left", Py - Protate);
        telemetry.addData("Back Left", Px - Protate);
        telemetry.addData("Back Right", Py + Protate);
        telemetry.addData("Front Right", Px + Protate);

        // to make sure encoder wires are connected
        telemetry.addData("m0 position", m0.getCurrentPosition());
        telemetry.addData("m1 position", m1.getCurrentPosition());
        telemetry.addData("m2 position", m2.getCurrentPosition());
        telemetry.addData("m3 position", m3.getCurrentPosition());

        double frontLeftPower = (Py - Protate) * 2;
        double backLeftPower = (Px - Protate) * 2;
        double frontRightPower = (Px + Protate) * 2;
        double backRightPower = (Py + Protate) * 2;

        if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1 ) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        if (!slowmo) {
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);
            frontRightMotor.setPower(frontRightPower);
        }
        if (slowmo){
            frontLeftMotor.setPower((Py - Protate)*.2);
            backLeftMotor.setPower((Px - Protate)*.2);
            backRightMotor.setPower((Py + Protate)*.2);
            frontRightMotor.setPower((Px + Protate)*.2);
        }
    }
    public void resetAngle(){
        if(gamepad1.y){
            reset_angle = getHeading() + reset_angle;
        }
    }
    public double getHeading(){
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if(heading < -180) {
            heading = heading + 360;
        }
        else if(heading > 180){
            heading = heading - 360;
        }
        heading = heading - reset_angle;
        return heading;
    }
    public void setSlowmoToggle(){
        //Slowmo toggle
        if(gamepad1.b)
        {
            if (slowmoToggle ==0)
            {
                slowmoToggle = 1;
            }
            if (slowmoToggle == 2)
            {
                slowmoToggle = 3;
            }

        }
        else
        {
            if (slowmoToggle == 1)
            {
                slowmoToggle = 2;
                slowmo = true;
            }

            if (slowmoToggle == 3)
            {
                slowmoToggle = 0;
                slowmo = false;
            }
        }
    }
    public void stoneClawControl(){
        //stoneClaw
        if(gamepad1.a)
        {
            if(clawToggle == 0)
            {
                clawToggle = 1;
            }
            if(clawToggle == 2)
            {
                clawToggle = 3;
            }
        }
        else {
            if (clawToggle == 1)
            {
                clawToggle = 2;
                stoneClaw.setPosition(.7);
            }

            if (clawToggle == 3)
            {
                clawToggle = 0;
                stoneClaw.setPosition(.05);
            }
        }
    }
    public void foundationClawControl(){
        //Foundation servo
        if(gamepad1.right_bumper)
        {
            if(foundationToggle == 0)
            {
                foundationToggle = 1;
            }
            if(foundationToggle == 2)
            {
                foundationToggle = 3;
            }
        }
        else {
            if (foundationToggle == 1)
            {
                foundationToggle = 2;
                foundationClaw.setPosition(.7);
            }
            if (foundationToggle == 3)
            {
                foundationToggle = 0;
                foundationClaw.setPosition(0.25);
            }
        }
    }
    public void liftControl(){
        //lift
        if(gamepad1.right_trigger > 0.1)
        {
            lift.setPower(-.5);
        }
        if(gamepad1.left_trigger > 0.1)
        {
            lift.setPower(.5);
        }
        else
        {
            lift.setPower(0);
        }
    }
}
