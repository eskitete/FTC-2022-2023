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
@TeleOp (name="redTeleStrafe", group="6455")
public class redTeleStrafe extends LinearOpMode {

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

    //public static PIDCoefficients LIFT_PID = new PIDCoefficients(0.01,0,0);
    //public static PIDFController LIFT_CONTROLLER = new PIDFController(LIFT_PID);

    //constants for lift movement
    public static int target = 300;
    public static double kG = 0.3;

    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0;
    public static double downConstant = 0.03;

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
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftL.setDirection(DcMotor.Direction.REVERSE);

        //all motors on the right side are reversed
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        controller = new PIDController(p, i , d);
        controller.setPID(p, i, d);

        boolean up = false;
        boolean down = false;
        boolean still = false;

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();


        while(opModeIsActive()) {
            /*if(gamepad2.y){
                up = true;
                down = false;
            }
            if(gamepad2.a){
                up = false;
                down = true;
            }
            if(up){
                int position = liftL.getCurrentPosition();
                double power = controller.calculate(position, target) + kG;
                liftR.setPower(power);
                liftL.setPower(power);
            }
            else if(down){
                liftR.setPower(downConstant);
                liftL.setPower(downConstant);
            }
            double setPower = controller.calculate(liftR.getCurrentPosition(), target);
            telemetry.addData("power", setPower);
            liftR.setPower(setPower + kG);
            liftL.setPower(setPower + kG);
            */


            // Drivetrain
             double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
             double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
             double rightX = -gamepad1.right_stick_x/1.5;
             final double v1 = r * Math.cos(robotAngle) + rightX;
             final double v2 = r * Math.sin(robotAngle) - rightX;
             final double v3 = r * Math.sin(robotAngle) + rightX;
             final double v4 = r * Math.cos(robotAngle) - rightX;
             FL.setPower(v1 * 0.7);
             FR.setPower(v2 * 0.7);
             BL.setPower(v3 * 0.7);
             BR.setPower(v4 * 0.7);

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

/*            if(gamepad2.y){
                liftR.setTargetPosition(350);
                liftL.setTargetPosition(350);
                liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftL.setPower(0.5);
                liftR.setPower(0.5);
            }*/
            //auto lift movement to position
                //top position
            if(gamepad2.y){
                while(opModeIsActive()){
                    double setPower = controller.calculate(liftR.getCurrentPosition(), 340);
                    telemetry.addData("power", setPower);
                    liftR.setPower(setPower + kG);
                    liftL.setPower(setPower + kG);
                    if(Math.abs(controller.getPositionError()) < 10){
                        still = true;
                        break;
                    }
                }
            }
                //middle position
            if(gamepad2.b){
                while(opModeIsActive()){
                    double setPower = controller.calculate(liftR.getCurrentPosition(), 130);
                    telemetry.addData("power", setPower);
                    liftR.setPower(setPower + kG);
                    liftL.setPower(setPower + kG);
                    if(Math.abs(controller.getPositionError()) < 10){
                        still = true;
                        break;
                    }

                }
            }
                //lower position
            if(gamepad2.x) {
                while (opModeIsActive()) {
                    double setPower = controller.calculate(liftR.getCurrentPosition(), 90);
                    telemetry.addData("power", setPower);
                    liftR.setPower(setPower + kG);
                    liftL.setPower(setPower + kG);
                    if (Math.abs(controller.getPositionError()) < 10) {
                        still = true;
                        break;
                    }
                }
            }
            if(still){
                liftL.setPower(kG);
                liftR.setPower(kG);
            }
            //lower lift to bottom slowly*/
            if(gamepad2.a){
                still = false;
                liftL.setPower(downConstant);
                liftR.setPower(downConstant);
            }

/*          if(gamepad2.y){
                up = true;
                down = false;
                target = 400;
            }
            if(gamepad2.a){
                up = false;
                down = true;
                target = 70;
            }
            if(up){
                LIFT_CONTROLLER.setTargetPosition(target);
                double setPower = LIFT_CONTROLLER.update(liftR.getCurrentPosition());
                liftR.setPower(setPower + kG);
                liftL.setPower(setPower + kG);
            }
            else if(down){
                LIFT_CONTROLLER.setTargetPosition(target);
                double setPower = LIFT_CONTROLLER.update(liftR.getCurrentPosition());
                liftR.setPower(setPower + kG);
                liftL.setPower(setPower + kG);
            }*/
/*            if(gamepad2.y){
                while(opModeIsActive()){
                    LIFT_CONTROLLER.setTargetPosition(400);
                    double setPower = LIFT_CONTROLLER.update(liftR.getCurrentPosition());
                    liftR.setPower(setPower + kG);
                    liftL.setPower(setPower + kG);
                    telemetry.addData("lift L = ", liftL.getCurrentPosition());
                    telemetry.addData("Lift R = ", liftR.getCurrentPosition());
                    telemetry.update();
                }
            }
            if(gamepad2.a){
                while(opModeIsActive()){
                    LIFT_CONTROLLER.setTargetPosition(70);
                    double setPower = LIFT_CONTROLLER.update(liftR.getCurrentPosition());
                    liftR.setPower(setPower + kG);
                    liftL.setPower(setPower + kG);
                    telemetry.addData("lift L = ", liftL.getCurrentPosition());
                    telemetry.addData("Lift R = ", liftR.getCurrentPosition());
                    telemetry.update();
                }
            }*/
/*            if(gamepad2.a) {
                target = 70;
            }

            if(gamepad2.y){
                target = 400;
            }
            LIFT_CONTROLLER.setTargetPosition(target);
            double setPower = LIFT_CONTROLLER.update(liftR.getCurrentPosition());
            liftR.setPower(setPower + kG);
            liftL.setPower(setPower + kG);
*/
            telemetry.addData("Left lift", liftL.getCurrentPosition());
            telemetry.addData("Right lift", liftR.getCurrentPosition());
            telemetry.update();



        }
    }
}

