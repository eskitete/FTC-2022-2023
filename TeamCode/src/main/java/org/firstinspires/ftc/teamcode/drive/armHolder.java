package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class armHolder extends LinearOpMode {
    private DcMotorEx liftR;
    private DcMotorEx liftL;
    FtcDashboard dashboard;
    //public static PIDFCoefficients LIFT_PIDF = new PIDFCoefficients(0, 0, 0, 0);
    public static PIDCoefficients LIFT_PID = new PIDCoefficients(0.01,0,0);
    public static int target = 400;
    public static double kG = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        PIDFController LIFT_CONTROLLER = new PIDFController(LIFT_PID);
        liftR = (DcMotorEx) hardwareMap.dcMotor.get("liftR");
        liftL = (DcMotorEx) hardwareMap.dcMotor.get("liftL");

        liftL.setDirection(DcMotor.Direction.REVERSE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        LIFT_CONTROLLER.setTargetPosition(target);
        //LIFT_CONTROLLER.setTargetAcceleration(9.81);

        while(opModeIsActive()){
            LIFT_CONTROLLER.setTargetPosition(target);
            double setPower = LIFT_CONTROLLER.update(liftR.getCurrentPosition());
            liftR.setPower(setPower + kG);
            liftL.setPower(setPower + kG);
            telemetry.addData("lift L = ", liftL.getCurrentPosition());
            telemetry.addData("Lift R = ", liftR.getCurrentPosition());
            telemetry.update();
        }

/*        liftR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LIFT_PIDF);
        liftL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LIFT_PIDF);

        liftL.setTargetPosition(target);
        liftR.setTargetPosition(target);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();
        waitForStart();

        liftL.setPower(power);
        liftR.setPower(power);
        while (opModeIsActive() && ((Math.abs(liftL.getCurrentPosition())) < (Math.abs(liftL.getTargetPosition()))) && ((Math.abs(liftR.getCurrentPosition())) < (Math.abs(liftR.getTargetPosition())))) {

            telemetry.addData("lift L = ", liftL.getCurrentPosition());
            telemetry.addData("Lift R = ", liftR.getCurrentPosition());
            telemetry.update();
        }

        liftR.setPower(0);
        liftL.setPower(0);*/

    }
}

