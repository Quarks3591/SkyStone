package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class TankDrive extends OpMode
{
    DcMotor FrontRight;
    DcMotor BackRight;
    DcMotor FrontLeft;
    DcMotor BackLeft;

    public void init()
    {
        FrontRight = hardwareMap.dcMotor.get("Front_Right");
        BackRight = hardwareMap.dcMotor.get("Back_Right");
        FrontLeft = hardwareMap.dcMotor.get("Front_Left");
        BackLeft = hardwareMap.dcMotor.get("Back_Left");

        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void loop()
    {
        //Left Control
        if(gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1)
        {
            FrontLeft.setPower(gamepad1.left_stick_y);
            BackLeft.setPower(gamepad1.left_stick_y);
        }

        //Right Control
        if(gamepad1.right_stick_y > 0.1 || gamepad1.right_stick_y < -0.1)
        {
            FrontRight.setPower(gamepad1.right_stick_y);
            BackRight.setPower(gamepad1.right_stick_y);
        }

        //Strafe Left
        if(gamepad1.dpad_left)
        {
            FrontLeft.setPower(1.0);
            BackLeft.setPower(-1.0);
            FrontRight.setPower(1.0);
            BackRight.setPower(-1.0);
        }

        //Strafe Right
        if(gamepad1.dpad_right)
        {
            FrontLeft.setPower(-1.0);
            BackLeft.setPower(1.0);
            FrontRight.setPower(-1.0);
            BackRight.setPower(1.0);
        }

        //Default
        else{
            FrontLeft.setPower(0.0);
            BackLeft.setPower(0.0);
            FrontRight.setPower(0.0);
            BackRight.setPower(0.0);
        }
    }
}
