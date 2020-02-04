package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.teamcode.Gyro;

@Disabled
public class Auto2019noFunctions extends LinearOpMode {
    //Wheels
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;

    //Extra Motors/Servos
    DcMotor lift;
    Servo claw;
    Servo foundationBoi;
    BNO055IMU imu;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    DriveTrain robot = new DriveTrain(this, new Gyro(this));

    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // 5202 Series Yellow Jacket Planetary Gear Motor
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.5;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.3;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    //init input variables
    private String color = "Red";
    private String side = "Left";
    private String target = "Center";

    @Override
    public void runOpMode(){
        //Wheel Initiation
        frontRightMotor = hardwareMap.dcMotor.get("front_right");
        backRightMotor = hardwareMap.dcMotor.get("back_right");
        frontLeftMotor = hardwareMap.dcMotor.get("front_left");
        backLeftMotor = hardwareMap.dcMotor.get("back_left");

        //Extra Initiation
        lift = hardwareMap.dcMotor.get("lift");
        claw = hardwareMap.servo.get("stoneClaw");
        foundationBoi = hardwareMap.servo.get("foundation");
        imu = hardwareMap.get(BNO055IMU .class,"gyro");

        //set drivetrain direction
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set init power
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        //servos
        claw.setPosition(.4);
        foundationBoi.setPosition(.7);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        //make sure gyro is calibrated
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("gyro calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        //side/color input for match
        while (!opModeIsActive()) {
            if(gamepad1.x || gamepad2.x) color = "Blue";
            if (gamepad1.b || gamepad2.b) color = "Red";
            if (gamepad1.y) color = "yellow";
            telemetry.addData("Color {Blue (X), Red (B)}", color);

            telemetry.update();
        }

        //wait for start button
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        //loop until end of autonomous period
        while (opModeIsActive()) {
            correction = checkDirection();

            telemetry.addData("1 gyro heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();
            if (color == "Red")
            {
                gyroDrive(DRIVE_SPEED,30,0 );
                gyroTurn(TURN_SPEED,-90);
                gyroDrive(DRIVE_SPEED,-36,-90);
                sleep(500);
                foundationBoi.setPosition(.2);
                sleep(100);
                gyroDrive(DRIVE_SPEED,60,-90);
                sleep(100);
                frontLeftMotor.setPower(-.5);
                backLeftMotor.setPower(.5);
                frontRightMotor.setPower(.5);
                backRightMotor.setPower(-.5);
                sleep(1000);
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                /*
                gyroDrive(DRIVE_SPEED, 12, 0);
                sleep(100);
                gyroTurn(TURN_SPEED, 90);
                sleep(100);
                gyroDrive(DRIVE_SPEED, 48, 90);
                sleep(100);
                gyroTurn(TURN_SPEED, 180);
                sleep(100);
                foundationClaw.setPosition(.2);
                sleep(100);

                frontLeftMotor.setPower(.5);
                backLeftMotor.setPower(-.5);
                frontRightMotor.setPower(-.5);
                backRightMotor.setPower(.5);

                sleep(1500);

                frontLeftMotor.setPower(0.0);
                backLeftMotor.setPower(0.0);
                frontRightMotor.setPower(0.0);
                backRightMotor.setPower(0.0);

                sleep(100);
                foundationClaw.setPosition(.7);
                sleep(100);
                gyroDrive(-DRIVE_SPEED, 24, 0);
                 */
                sleep(100000);
            }
            if(color == "Blue")
            {
                gyroDrive(DRIVE_SPEED,44,0 );
                gyroTurn(TURN_SPEED,90);
                gyroDrive(DRIVE_SPEED,36,90);
                sleep(500);
                foundationBoi.setPosition(.2);
                sleep(100);
                gyroDrive(DRIVE_SPEED,-36,90);
                sleep(100);
                frontLeftMotor.setPower(.5);
                backLeftMotor.setPower(-.5);
                frontRightMotor.setPower(-.5);
                backRightMotor.setPower(.5);
                sleep(1000);
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                /*
                gyroDrive(DRIVE_SPEED, 12, 0);
                sleep(100);
                gyroTurn(TURN_SPEED, -90);
                sleep(100);
                gyroDrive(DRIVE_SPEED, 48, 90);
                sleep(100);
                gyroTurn(TURN_SPEED, 180);
                sleep(100);
                foundationClaw.setPosition(.3);
                sleep(100);

                frontLeftMotor.setPower(.5);
                backLeftMotor.setPower(-.5);
                frontRightMotor.setPower(-.5);
                backRightMotor.setPower(.5);

                sleep(1500);

                frontLeftMotor.setPower(0.0);
                backLeftMotor.setPower(0.0);
                frontRightMotor.setPower(0.0);
                backRightMotor.setPower(0.0);

                sleep(100);
                foundationClaw.setPosition(.7);
                sleep(100);
                gyroDrive(-DRIVE_SPEED, 24, 0);

                 */
                sleep(100000);
            }
            if (color == "yellow"){
                gyroDrive(DRIVE_SPEED, 12, 0);
                sleep(1000000000);
            }
        }
    }





    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the gyro works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
    private double checkDirection()
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

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart gyro movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        frontLeftMotor.setPower(leftPower);
        backLeftMotor.setPower(leftPower);
        frontRightMotor.setPower(rightPower);
        backRightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }



    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed, double distance, double angle) {

        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);

            int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + moveCounts;
            int newBackLeftTarget = backLeftMotor.getCurrentPosition() + moveCounts;
            int newFrontRightTarget = frontRightMotor.getCurrentPosition() + moveCounts;
            int newBackRightTarget = backRightMotor.getCurrentPosition() + moveCounts;

            setTargetPosition(moveCounts);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            frontLeftMotor.setPower(speed);
            backLeftMotor.setPower(speed);
            frontRightMotor.setPower(speed);
            backRightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                frontLeftMotor.setPower(leftSpeed);
                backLeftMotor.setPower(leftSpeed);
                frontRightMotor.setPower(rightSpeed);
                backRightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Actual",  "%7d:%7d", frontLeftMotor.getCurrentPosition(),
                        frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            frontLeftMotor.setPower(0.0);
            backLeftMotor.setPower(0.0);
            frontRightMotor.setPower(0.0);
            backRightMotor.setPower(0.0);

            // Turn off RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        frontLeftMotor.setPower(leftSpeed);
        backLeftMotor.setPower(leftSpeed);
        frontRightMotor.setPower(rightSpeed);
        backRightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        getAngle();

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - globalAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void setTargetPosition(int moveCounts) {
        int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + moveCounts;
        int newBackLeftTarget = backLeftMotor.getCurrentPosition() + moveCounts;
        int newFrontRightTarget = frontRightMotor.getCurrentPosition() + moveCounts;
        int newBackRightTarget = backRightMotor.getCurrentPosition() + moveCounts;

        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        frontRightMotor.setTargetPosition(newFrontRightTarget);
        backRightMotor.setTargetPosition(newBackRightTarget);
    }
}
