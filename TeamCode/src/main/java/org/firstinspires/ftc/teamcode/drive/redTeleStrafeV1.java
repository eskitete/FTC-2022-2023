package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp (name="redTeleV1", group="6455")
public class redTeleStrafeV1 extends LinearOpMode {

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
    public static double kG = 0.02;
    public static double downConstant = 0.03;

    enum State{
        MANUAL,
        DEFAULT,
        LIFT,
        HOLD,
        DOWN
    }
    State currState = State.DEFAULT;
    @Override
    public void runOpMode() throws InterruptedException {
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
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftL.setDirection(DcMotor.Direction.REVERSE);

        //all motors on the right side are reversed
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive()){
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

            //SET TARGET
            if(gamepad2.y){
                target = 400;
                currState = State.LIFT;
            }
            if(gamepad2.a){
                target = 200;
                currState = State.LIFT;
            }
            if(gamepad2.x){
                currState = State.DOWN;
            }

            //Fine tuned control
            if(gamepad2.left_stick_y != 0) {
                liftR.setPower(-.7 * gamepad2.left_stick_y);
                liftL.setPower(-.7 * gamepad2.left_stick_y);
            }
            else if(gamepad2.left_stick_y == 0){
                liftR.setPower(kG);
                liftL.setPower(kG);
            }

            //Intake movement
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
                case LIFT:{
                    if(Math.abs(liftL.getCurrentPosition() - target) < 20 || Math.abs(liftL.getCurrentPosition() - target) < 20){
                        currState = State.HOLD;
                    }
                    else{
                        if(liftL.getCurrentPosition() < target){
                            liftL.setPower(-0.7);
                            liftR.setPower(-0.7);
                        }
                        else{
                            liftL.setPower(0.7);
                            liftR.setPower(0.7);
                        }
                    }
                    break;
                }
                case HOLD:{
                    liftR.setPower(kG);
                    liftL.setPower(kG);
                    break;
                }
                case DOWN:{
                    liftR.setPower(downConstant);
                    liftL.setPower(downConstant);
                }
            }
            telemetry.addData("liftL curr", liftL.getCurrentPosition());
            telemetry.addData("state", currState);
            telemetry.update();

        }
    }
}
