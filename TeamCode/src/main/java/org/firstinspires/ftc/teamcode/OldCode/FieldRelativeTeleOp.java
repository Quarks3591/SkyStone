package org.firstinspires.ftc.teamcode.OldCode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
@Disabled
public class FieldRelativeTeleOp extends OpMode
{
    //Wheels
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;

    //Extra Motors/Servos
    DcMotor lift;
    Servo claw;
    Servo foundationBoi;
    BNO055IMU gyro;

    int clawToggle = 0;
    int foundationToggle = 0;
    int slowmoToggle = 0;

    boolean left_stick_pressed = false;
    boolean robotCentric = false;
    boolean slowmo = false;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle;

    public void init()
    {
        //Wheel Initiation
        frontRightMotor = hardwareMap.dcMotor.get("front_right");
        backRightMotor = hardwareMap.dcMotor.get("back_right");
        frontLeftMotor = hardwareMap.dcMotor.get("front_left");
        backLeftMotor = hardwareMap.dcMotor.get("back_left");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Extra Initiation
        lift = hardwareMap.dcMotor.get("lift");
        claw = hardwareMap.servo.get("stoneClaw");
        foundationBoi = hardwareMap.servo.get("foundation");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        gyro.initialize(parameters);

        claw.scaleRange(0, 1);

        while (gyro.isGyroCalibrated()) { }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("gyro calib status", gyro.getCalibrationStatus().toString());
        telemetry.update();
    }
    public void loop()
    {
        getAngle();
        telemetry.addData("heading", globalAngle);
        telemetry.update();

        double xRaw = scaleInput(gamepad1.left_stick_x);
        double yRaw = -scaleInput(gamepad1.left_stick_y); // Remember, this is reversed!
        double yDirection = yRaw * Math.sin(globalAngle) + xRaw *Math.cos(globalAngle);
        double xDirection = (xRaw * Math.cos(globalAngle) - yRaw * Math.sin(globalAngle))* 1.5;
        double rx = gamepad1.right_stick_x;

        double frontLeftPower = yDirection + xDirection + rx;
        double backLeftPower = yDirection - xDirection + rx;
        double frontRightPower = yDirection - xDirection - rx;
        double backRightPower = yDirection + xDirection - rx;

        // Put powers in the range of -1 to 1 only if they aren't already (not
        // checking would cause us to always drive at full speed)

        if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1) {
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

        frontLeftPower = scaleInput(frontLeftPower);
        backLeftPower = scaleInput(backLeftPower);
        frontRightPower = scaleInput(frontRightPower);
        backRightPower = scaleInput(backRightPower);

        if (!slowmo)
        {
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
        if (slowmo)
        {
            frontLeftMotor.setPower(frontLeftPower*.2);
            backLeftMotor.setPower(backLeftPower*.2);
            frontRightMotor.setPower(frontRightPower*.2);
            backRightMotor.setPower(backRightPower*.2);
        }

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
                claw.setPosition(.7);
            }

            if (clawToggle == 3)
            {
                clawToggle = 0;
                claw.setPosition(0);
            }
        }
        if(gamepad1.x)
        {
            claw.setPosition(0);
        }
        if(gamepad1.y)
        {
            claw.setPosition(.7);
        }

        //lift
        if(gamepad1.dpad_up)
        {
            lift.setPower(1.0);
        }
        if(gamepad1.dpad_down)
        {
            lift.setPower(-1.0);
        }
        else
        {
            lift.setPower(0);
        }

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
                foundationBoi.setPosition(.6);
            }
            if (foundationToggle == 3)
            {
                foundationToggle = 0;
                foundationBoi.setPosition(0.3);
            }
        }
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
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the gyro works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    double scaleInput(double dVal)
    {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0)
        {
            index = -index;
        }
        else if (index > 16)
        {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0)
        {
            dScale = -scaleArray[index];
        } else
        {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
