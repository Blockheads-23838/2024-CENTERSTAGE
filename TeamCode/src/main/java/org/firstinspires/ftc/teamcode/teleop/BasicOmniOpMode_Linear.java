package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.copySign;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Button;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private DcMotorEx lift = null;
    private DcMotorEx climberDownstairs = null;
    private DcMotorEx climberUpstairs = null;
    private IMU imu = null;
    private Servo servo = null;
    private Servo crossbow = null;
    private Servo autoHook = null;
    private TouchSensor liftLimit = null;
    double powercoef = 0.5;
    private double autoHookSetpoint = Constants.autoHookStowPosition;
    private boolean fieldCentric = false;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");

        imu = hardwareMap.get(IMU.class, "imu");

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        intake = hardwareMap.get(DcMotor.class, "intake");
        climberDownstairs = hardwareMap.get(DcMotorEx.class, "climber_downstairs");
        climberUpstairs = hardwareMap.get(DcMotorEx.class, "climber_upstairs");

        servo = hardwareMap.get(Servo.class, "servo");
        autoHook = hardwareMap.get(Servo.class, "auto_hook");
        crossbow = hardwareMap.get(Servo.class, "crossbow");

        liftLimit = hardwareMap.get(TouchSensor.class, "lift_limit");

        climberDownstairs.setDirection(Constants.motorDirections.get("climber_downstairs"));
        leftFrontDrive.setDirection(Constants.motorDirections.get("left_front"));
        leftBackDrive.setDirection(Constants.motorDirections.get("left_back"));
        rightFrontDrive.setDirection(Constants.motorDirections.get("right_front"));
        rightBackDrive.setDirection(Constants.motorDirections.get("right_back"));
        lift.setDirection(Constants.motorDirections.get("lift"));

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imu.resetYaw();

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() && !isStopRequested()) {
            if (gamepad2.a) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            telemetry.addData("lift position: ", lift.getCurrentPosition());
            telemetry.update();
        }

        runtime.reset();
        climberUpstairs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberUpstairs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servo.setPosition(0);
        autoHook.setPosition(Constants.autoHookStowPosition);
        crossbow.setPosition(0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // arm servo debug code
            // arm.setPower(gamepad2.right_stick_y * 1/4);
            // servo.setPosition(position);
            // intake.setPower(-gamepad2.right_trigger);

            // crossbow
            double crossbowSetpoint;
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                telemetry.addLine("crossbowing!!");
                crossbowSetpoint = Constants.crossbowFirePosition;
            } else {
                crossbowSetpoint = Constants.crossbowRestPosition;
            }
            telemetry.addData("crossbow setpoint", crossbowSetpoint);
            if (crossbowSetpoint != crossbow.getPosition()) crossbow.setPosition(crossbowSetpoint);

            if (gamepad1.b) imu.resetYaw();

            handleLift();

            HandleDrivetrain();
            telemetry.addData("lift limit status: ", liftLimit.isPressed());
            telemetry.update();
        }
    }

    public void handleLift() {
        // Lift stuff
        if (gamepad2.a) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        int liftPos = lift.getCurrentPosition();
        telemetry.addData("lift position: ", liftPos);
        double liftPower;
        if (gamepad2.left_bumper) liftPower = gamepad2.right_stick_y * 1500; // manual override; also former ticks/sec value
        else if (liftPos > Constants.TopLiftPosition && -gamepad2.right_stick_y < 0) liftPower = 0; // Upper limit
        else if (liftPos < Constants.groundLiftPosition && gamepad2.right_stick_y <= 0) liftPower = -liftPos*10; // Lower limit
        else if (gamepad2.right_stick_y < 0) liftPower = gamepad2.right_stick_y * (lift.getCurrentPosition()*3 + 200); // If we're going down, act as a P controller to reduce overshoot
        else liftPower = gamepad2.right_stick_y * 2500;
        lift.setVelocity(liftPower + 1);

        // Intake stuff
        if (gamepad2.right_trigger > 0.5) {
            intake.setPower(-1);
        } else {
            intake.setPower(gamepad2.left_trigger);
        }

        // Servo stuff
        double servoSetpoint;
        if (gamepad2.y) {
            servoSetpoint = 0.22; // dropping pixels; 0.19 is decent 14.57 12.22.23
        } else if (gamepad2.right_trigger > 0.5 /* ||
                (lift.getCurrentPosition() < Constants.ClearIntakeLiftPosition && -gamepad2.right_stick_y > 0) */){
            // put pan down if we're intaking or clearing the intake
            servoSetpoint = Constants.IntakingServoPosition;
        } else {
            // stow/carry position
            servoSetpoint = 0.09;
        }
        // sleep(20);
        telemetry.addData("servoSetpoint: ", servoSetpoint);
        telemetry.addData("Last servo setpoint: ", servo.getPosition());
        if (servo.getPosition() != servoSetpoint) servo.setPosition(servoSetpoint);

        // Climber stuff
        if ((climberDownstairs.getCurrentPosition() < Constants.climberDownstairsGoToZeroPosition
            && gamepad2.left_stick_y <= 0 && !gamepad2.left_bumper)) {
            // Add a zone where it P controls to horizontal
            climberDownstairs.setVelocity(-climberDownstairs.getCurrentPosition() * 3);
        } else {
            climberDownstairs.setVelocity(600 * gamepad2.left_stick_y + 1);
        }

        if (gamepad2.dpad_up) climberUpstairs.setVelocity(2400);
        else if (gamepad2.dpad_down) climberUpstairs.setVelocity(-2400);
        else climberUpstairs.setVelocity(0);

        if (gamepad2.a && gamepad2.dpad_up) autoHookSetpoint += 0.1;
        else if (gamepad2.a && gamepad2.dpad_down) autoHookSetpoint -= 0.1;
        if (autoHook.getPosition() != autoHookSetpoint) autoHook.setPosition(autoHookSetpoint);
        telemetry.addData("climber downstairs position: ", climberDownstairs.getCurrentPosition());

    }

    public void HandleDrivetrain() {
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

        if (gamepad1.y) fieldCentric = !fieldCentric;
        if (gamepad1.b) imu.resetYaw();
        telemetry.addData("FIELD_CENTRIC: ", fieldCentric);

        if(gamepad1.right_bumper) powercoef = 1;
        else if (gamepad1.left_trigger > 0.3) powercoef = 0.2;
        else powercoef = 0.4;

        double forward = -gamepad1.left_stick_y; /* Invert stick Y axis */
        double strafe = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        if (fieldCentric) {
            double gyro_radians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("gyro angle: ", gyro_radians);
            strafe = -strafe;
            double temp = forward * Math.cos(gyro_radians) +
                    strafe * Math.sin(gyro_radians);
            strafe = -forward * Math.sin(gyro_radians) +
                    strafe * Math.cos(gyro_radians);
            strafe *= -1;
            forward = temp;

            /* At this point, Joystick X/Y (strafe/forwrd) vectors have been */
            /* rotated by the gyro angle, and can be sent to drive system */
        }


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = powercoef * (forward + strafe + yaw);
        double rightFrontPower = powercoef * (forward - strafe - yaw);
        double leftBackPower = powercoef * (forward - strafe + yaw);
        double rightBackPower = powercoef * (forward + strafe - yaw);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        telemetry.addData("left front pos: ", leftFrontDrive.getCurrentPosition());
        telemetry.addData("left back pos: ", leftBackDrive.getCurrentPosition());
        telemetry.addData("right front pos: ", rightFrontDrive.getCurrentPosition());
        telemetry.addData("right back pos: ", rightBackDrive.getCurrentPosition());

    }
}
