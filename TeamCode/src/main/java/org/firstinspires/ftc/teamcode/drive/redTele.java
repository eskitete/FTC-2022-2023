package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp (name="redTele", group="6455")
public class redTele extends LinearOpMode {

    // initialize the drive motors
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;

    // initialize the lift servos/motors
    private DcMotor liftR;
    private DcMotor liftL;


    // initialize the four-bar servos servos/motors
    private Servo fBarServoR;
    private Servo fBarServoL;

    // initialize the intake servo
    private Servo intake;

    //constants for lift movement
    public static int target = 300;
    public static double kG = 0.3;
    public static double downConstant = 0.03;

    enum State{
        DEFAULT,
        LIFT,
        DOWN
    }
    State currState = State.DEFAULT;
    public void runOpMode() {
        // maps all motors and servos to the rev hup config
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        fBarServoR = hardwareMap.servo.get("fBarServoR");
        fBarServoL = hardwareMap.servo.get("fBarServoL");

        liftR = hardwareMap.dcMotor.get("liftR");
        liftL = hardwareMap.dcMotor.get("liftL");

        intake = hardwareMap.servo.get("intake");

        //sets motors to brake when stopped
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftL.setDirection(DcMotor.Direction.REVERSE);

        //all motors on the right side are reversed
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive()) {
            // Drivetrain
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x/1.5;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            //Slowed down power a bit
            FL.setPower(v1 * 0.5);
            FR.setPower(v2 * 0.5);
            BL.setPower(v3 * 0.5);
            BR.setPower(v4 * 0.5);

            // controller 2 right joystick y-axis moves the lift manually
            if(gamepad2.left_stick_y!=0) {
                liftR.setPower(-.7 * gamepad2.left_stick_y);
                liftL.setPower(-.7 * gamepad2.left_stick_y);
            }
            //four bar movement
            if(gamepad2.right_bumper){
                fBarServoL.setPosition(1);
                fBarServoR.setPosition(0);
            }
            else {if(gamepad2.right_trigger!=0){
                fBarServoL.setPosition(0.1);
                fBarServoR.setPosition(.95);
            }
            }
            //intake movement
            if(gamepad2.left_trigger!=0){
                intake.setPosition(.3);
            }
            if (gamepad2.left_bumper){
                intake.setPosition(.05);
            }
            if (gamepad2.dpad_up){
                intake.setPosition(.5);
            }
            if (gamepad2.dpad_up){
                intake.setPosition(.45);
            }

            switch (currState){
                case DEFAULT:{
                    if(gamepad2.y){
                        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        liftL.setTargetPosition(350);
                        liftR.setTargetPosition(350);

                        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        liftR.setPower(-0.7);
                        liftL.setPower(-0.7);
                        currState = State.LIFT;
                    }
                    break;
                }
                case LIFT:{
                    if(liftL.getCurrentPosition() > liftL.getTargetPosition() && liftR.getCurrentPosition() > liftL.getTargetPosition()){
                        fBarServoL.setPosition(0.1);
                        fBarServoR.setPosition(0.95);
                    }

                    //DROP button goes here
                    if(gamepad2.x){
                        currState = State.DOWN;
                    }
                    break;
                }
                case DOWN:{
                    liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    liftR.setPower(downConstant);
                    liftL.setPower(downConstant);

                    if(liftR.getCurrentPosition() < 50 || liftL.getCurrentPosition() < 50){
                        currState = State.DEFAULT;
                    }
                    break;
                }
            }

            telemetry.addData("Left lift", liftL.getCurrentPosition());
            telemetry.addData("Right lift", liftR.getCurrentPosition());
            telemetry.update();
        }
    }
}


