package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by User on 10/17/2017.
 */

public class DriveTrain {

    //create drive train objects
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    public DcMotor lift;
    public Servo stoneClaw;
    public Servo foundationClawRight;
    public Servo foundationClawLeft;
    public BNO055IMU imu;

    public Gyro gyro;

    Orientation lastAngles = new Orientation();
    double globalAngle, correction;


    //init other objects
    HardwareMap hwm = null;
    DigitalChannel digitalTouch;  // Hardware Device Object

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    static final double COUNTS_PER_MOTOR_REV = 383.6;    // 5202 Series Yellow Jacket Planetary Gear Motor
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "Ae3U3f7/////AAABmftjnDbpWU1NrrIH5GCO3I02UT5Evbls1oCi" +
            "h16KSa7n9Z7Dn8Wd5AjSP/uPd0TiNhacdRt+4u3wDwR2LDXbrfikvG+KmNQwIZXPo7cNk3mJ00romkPNdPvlbu9" +
            "h9peSwAl6MbI1pHZm25JXmKe0rr+U2IkGCecjy0IH+ph+364qjyzYR7NrPw8JPCyIMFV19CPZsIZBL4iq4ftgZy" +
            "+R7/pOj8SssVqLn+ldPo4EsR0SG5piGGrpvRboltxo5u8UslDfO1dKAyJSan/cIh+EGShbbwGV/HkERrMYa3oVFE" +
            "PQStJpS8uJumVESEUqcJPTkmx5huO/1wCemG8uRiqQogGt17VanwbaXOZaNWQzKykL";

    LinearOpMode opmode;

    // Constructor
    public DriveTrain(LinearOpMode opmode, Gyro gyro) {
        this.opmode = opmode;
        this.gyro = gyro;
    }

    //hardware init method
    public void init(HardwareMap inhwm) {
        hwm = inhwm;

        //Wheel Initiation
        frontRightMotor = hwm.get(DcMotor.class, "front_right");
        backRightMotor = hwm.get(DcMotor.class, "back_right");
        frontLeftMotor = hwm.get(DcMotor.class, "front_left");
        backLeftMotor = hwm.get(DcMotor.class, "back_left");

        //set drivetrain direction
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Extra Initiation
        lift = hwm.get(DcMotor.class, "lift");
        stoneClaw = hwm.get(Servo.class, "claw");
        foundationClawLeft = hwm.get(Servo.class, "foundationLeft");
        foundationClawRight = hwm.get(Servo.class, "foundationRight");

        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //servos
        stoneClaw.setPosition(0);
        foundationClawRight.setPosition(.2);
        foundationClawLeft.setPosition(.7);

        //touch sensor
        digitalTouch = hwm.get(DigitalChannel.class, "digital_touch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
    }

    //Reset all drive train motors
    public void stopAndReset() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //set all drive train motors to run using encoders
    public void runUsingEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //method to drive forward (positive value) or backwards (negative value)
    public void drive(double power) {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public void leftDrive(double power) {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
    }

    public void rightDrive(double power) {
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    //stops the robot
    public void stopMotors() {
        drive(0.0);
    }

    //method to rotate to the left (counter-clockwise)
    public void turnLeft(double power) {
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    //reverse above method to turn to the right (clockwise)
    public void turnRight(double power) {
        turnLeft(-power);
    }

    //method allowing mecanum drive train to strafe left
    public void strafeLeft(double power) {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(power);
    }

    //reversing above method allows strafing right
    public void strafeRight(double power) {
        strafeLeft(-power);
    }

    public boolean allMotorsAreBusy() {
        return (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy());
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed, double distance, double angle) {

        int moveCounts;
        double max, leftSpeed, rightSpeed, error, steer;

        // Determine new target position, and pass to motor controller
        moveCounts = (int) (distance * COUNTS_PER_INCH);

        int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + moveCounts;
        int newBackLeftTarget = backLeftMotor.getCurrentPosition() + moveCounts;
        int newFrontRightTarget = frontRightMotor.getCurrentPosition() + moveCounts;
        int newBackRightTarget = backRightMotor.getCurrentPosition() + moveCounts;

        setTargetPosition(moveCounts);

        runToPosition();

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        drive(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (allMotorsAreBusy()) {

            // adjust relative speed based on heading error.
            error = gyro.getError(angle);
            steer = gyro.getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            leftDrive(leftSpeed);
            rightDrive(rightSpeed);


            // Display drive status for the driver.
            opmode.telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
            opmode.telemetry.addData("Target", "%7d:%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
            opmode.telemetry.addData("Actual", "%7d:%7d", frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
            opmode.telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
            opmode.telemetry.update();

        }

        // Stop all motion;
        stopMotors();

        // Turn off RUN_TO_POSITION
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opmode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opmode.telemetry.update();
        }
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

    boolean onHeading(double speed, double angle, double PCoeff) {
        boolean onTarget = false;
        double leftSpeed, rightSpeed;
        double steer, error;

        error = gyro.getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = gyro.getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftDrive(leftSpeed);
        rightDrive(rightSpeed);

        // Display it for the driver.
        opmode.telemetry.addData("Target", "%5.2f", angle);
        opmode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opmode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hwm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwm.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        VuforiaTrackables trackables = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable skyStone = trackables.get(0);
        skyStone.setName("skyStoneVuMark"); // can help in debugging; otherwise not necessary

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(trackables);

        trackables.activate();
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hwm.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwm.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}