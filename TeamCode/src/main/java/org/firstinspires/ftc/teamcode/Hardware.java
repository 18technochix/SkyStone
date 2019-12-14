package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware
{
    /* Public OpMode members. */
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor rightLift = null;
    public DcMotor leftLift = null;
    public DcMotor wrist = null;
    public Servo hands = null;
    public DcMotor slide = null;
    public Servo tail = null;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double leftPower;
    double rightPower;


    LinearOpMode opMode = null;
     Telemetry telemetry = null;

// constants

    int COUNTS_PER_INCH = 100;
    double s = .5;






    /* local OpMode members. */
     HardwareMap hwMap = null;
     ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    public Hardware(LinearOpMode _opMode){
        opMode = _opMode;
        telemetry = _opMode.telemetry;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //hubA
        leftBackDrive  = hwMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightLift = hwMap.get(DcMotor.class, "right_lift");
        leftLift = hwMap.get(DcMotor.class, "left_lift");



        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //hubB
        //wrist arm hands slide
        wrist = hwMap.get(DcMotor.class, "wrist");
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide = hwMap.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        //lift
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);

        wrist.setDirection(DcMotor.Direction.FORWARD);

        slide.setDirection(DcMotor.Direction.FORWARD);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slide.setTargetPosition(0);
//        slide.setPower(.3);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftLift.setPower(0);
        rightLift.setPower(0);
        slide.setPower(0);
        wrist.setPower(0);

        // Define and initialize ALL installed servos.
        hands = hwMap.get(Servo.class, "hands");
        tail = hwMap.get(Servo.class, "tail");
        //sensors


        // imu setup

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



    }



    public void encoderDrive(double speedLift, double speedWrist,
                             double liftInches, double wristInches, double slideInches, double speedSlide) {
        int newLeftTarget;
        int newRightTarget;
        int newWristTarget;
        int newSlideTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftLift.getCurrentPosition() + (int)(liftInches * COUNTS_PER_INCH);
            newRightTarget = rightLift.getCurrentPosition() + (int)(liftInches * COUNTS_PER_INCH);
            newWristTarget = wrist.getCurrentPosition() + (int)(wristInches * COUNTS_PER_INCH);
            newSlideTarget = slide.getCurrentPosition() + (int)(slideInches * COUNTS_PER_INCH);
            leftLift.setTargetPosition(newLeftTarget);
            rightLift.setTargetPosition(newRightTarget);
            wrist.setTargetPosition(newWristTarget);
            slide.setTargetPosition(newSlideTarget);
//1680 ticks for slide

            // Turn On RUN_TO_POSITION
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            leftLift.setPower(Math.abs(speedLift));
            rightLift.setPower(Math.abs(speedLift));
            wrist.setPower(Math.abs(speedWrist));
            slide.setPower(Math.abs(speedSlide));

           /* while (opMode.opModeIsActive() && slide.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path2",  "Running at %7d",
//                        leftLift.getCurrentPosition(),
//                        rightLift.getCurrentPosition());
//                        wrist.getCurrentPosition();
                        slide.getCurrentPosition());
                telemetry.update();

                if(slide.isBusy()){
                    telemetry.addData("slide is busy", true);
                    telemetry.update();
                }
            }


            */

            // Stop all motion;
            leftLift.setPower(0);
            rightLift.setPower(0);
           // wrist.setPower(0);
            //slide.setPower(0);

            // Turn off RUN_TO_POSITION
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

}
