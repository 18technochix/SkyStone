package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="BigBoiArm", group="Linear Opmode")
public class BigBoiArm extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    int wristCounter = 0;
    int slideCounter = 0;
    int handsCounter = 0;
    Hardware robot = new Hardware(this);
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // get data from imu
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Drive

            double lx = gamepad2.left_stick_x;
            double ly = -gamepad2.left_stick_y;


            double rx = gamepad2.right_stick_x;
            double ry = -gamepad2.right_stick_y;

            double fl = (robot.s * Range.clip(ly + rx + lx, -1.0, 1.0));
            double  bl = (robot.s * Range.clip(ly + rx - lx, -1.0, 1.0));
            double  fr = (robot.s * Range.clip(ly - rx - lx, -1.0, 1.0));
            double br = (robot.s * Range.clip(ly - rx + lx, -1.0, 1.0));

            // Setup a variable for each drive wheel to save power level for telemetry


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            robot.leftFrontDrive.setPower(fl);
            telemetry.addData("leftFrontPower", fl);
            robot.leftBackDrive.setPower(bl);
            telemetry.addData("leftBackPower", bl);
           robot.rightFrontDrive.setPower(fr);
            telemetry.addData("rightFrontPower", fr);
            robot.rightBackDrive.setPower(br);
            telemetry.addData("rightBackPower", br);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();



            // Setup a variable for each drive wheel to save power level for telemetry
            double armPower = 0.33;
            double slidePower = .1;


            //moving hand
            if(gamepad1.right_bumper) {
                robot.hands.setPosition(44.);
            }
            if(gamepad1.right_trigger==1){
                robot.hands.setPosition(0);
            }

            //moving lift

            while(gamepad1.a){
                robot.encoderDrive(0.25, 0, 3, 0., 0, 0);

            }

            while(gamepad1.b){
                robot.encoderDrive(0.25, 0, -3, 0, 0, 0);


        }

            if(gamepad1.dpad_left) {
                telemetry.addData("position", robot.slide.getCurrentPosition());
                telemetry.update();
            }

            //moving wrist

            if(gamepad1.x){
                robot.encoderDrive(0, .25, 0, 2, 0, 0);
            }

            if(gamepad1.y){
                robot.encoderDrive(0, .25, 0, -2, 0,0);
            }


            //slide moving (arm)
            if(gamepad1.left_bumper){
                robot.encoderDrive(0,0,0,0,2,.33);
            }


            if(gamepad1.left_trigger==1){
                robot.encoderDrive(0,0,0,0,-2,.33);
            }

            if(gamepad1.dpad_down) {
                robot.tail.setPosition(45);
            }

            if(gamepad1.dpad_up){
                robot.tail.setPosition(0);
            }


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.


        }
    }
}
