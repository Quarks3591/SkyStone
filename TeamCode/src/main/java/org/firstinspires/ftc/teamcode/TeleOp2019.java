package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOp2019 extends OpMode
{
    //Wheels
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;

    //Extra Motors/Servos
    DcMotor lift;
    Servo stoneClaw;
    Servo foundationClawLeft;
    Servo foundationClawRight;
    int clawToggle = 0;
    int foundationToggle = 0;
    int slowmoToggle = 0;

    static final double COUNTS_PER_MOTOR_REV = 383.6;    // 5202 Series Yellow Jacket Planetary Gear Motor
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double SPOOL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    // Calculates number of encoder counts per inch
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (SPOOL_DIAMETER_INCHES * 3.1415);

    boolean slowmo = false;

    public static int BLOCK_HEIGHT = (int) (5 * COUNTS_PER_INCH); // sets block height in encoder counts
    int liftTargetUp = lift.getCurrentPosition() + BLOCK_HEIGHT;
    int liftTargetDown = lift.getCurrentPosition() - BLOCK_HEIGHT;

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
        stoneClaw = hardwareMap.servo.get("claw");
        foundationClawRight = hardwareMap.servo.get("foundationRight");
        foundationClawLeft = hardwareMap.servo.get("foundationLeft");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stoneClaw.scaleRange(0, 1);

        stoneClaw.setPosition(0);
        foundationClawRight.setPosition(0.2);
        foundationClawLeft.setPosition(0.7);
    }
    public void loop()
    {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x * 1.5;
        double rx = gamepad1.right_stick_x;

        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        // Put powers in the range of -1 to 1 only if they aren't already (not
        // checking would cause us to always drive at full speed)

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
                stoneClaw.setPosition(1);
            }

            if (clawToggle == 3)
            {
                clawToggle = 0;
                stoneClaw.setPosition(0);
            }
        }

        // Lift
        if (gamepad1.right_bumper)
        {
            lift.setTargetPosition(liftTargetUp);
        }
        if (gamepad1.left_bumper)
        {
            lift.setTargetPosition(liftTargetDown);
        }

        //Foundation stoneClaw
        if(gamepad1.x)
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
                foundationClawRight.setPosition(.7);
                foundationClawLeft.setPosition(.2);
            }
            if (foundationToggle == 3)
            {
                foundationToggle = 0;
                foundationClawRight.setPosition(0.2);
                foundationClawLeft.setPosition(0.7);
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
}