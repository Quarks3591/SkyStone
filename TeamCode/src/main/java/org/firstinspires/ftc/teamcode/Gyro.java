package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by User on 10/17/2017.
 */

public class Gyro {

    public BNO055IMU gyro;

    Orientation             lastAngles = new Orientation();
    public double                  globalAngle, robotError, robotSteer;

    //init other objects
    HardwareMap hwm = null;

    LinearOpMode opmode;

    // Constructor
    public Gyro(LinearOpMode opmode){
        this.opmode = opmode;
    }

    //hardware init method
    public void init(HardwareMap inhwm) {
        hwm = inhwm;

        gyro = hwm.get(BNO055IMU.class, "gyro");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        gyro.initialize(parameters);
    }

    //make sure gyro is calibrated
    public void checkCalibration() {
        while (!opmode.isStopRequested() && !gyro.isGyroCalibrated()) {
            opmode.sleep(50);
            opmode.idle();
        }
        opmode.telemetry.addData("Mode", "waiting for start");
        opmode.telemetry.addData("gyro calib status", gyro.getCalibrationStatus().toString());
        opmode.telemetry.update();
    }

    public double getError(double targetAngle) {

        getAngle();

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - globalAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        robotSteer = Range.clip(error * PCoeff, -1, 1);
        return robotSteer;
    }

    public double getAngle()
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

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

}