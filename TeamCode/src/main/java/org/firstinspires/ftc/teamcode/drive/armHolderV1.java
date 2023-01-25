package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class armHolderV1 extends LinearOpMode {
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0;
    FtcDashboard dashboard;
    public static int target = 300;
    public static double fG = 0.25;
    public static double downConstant = 0.03;
    private DcMotorEx liftR;
    private DcMotorEx liftL;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i , d);
        liftR = (DcMotorEx) hardwareMap.dcMotor.get("liftR");
        liftL = (DcMotorEx) hardwareMap.dcMotor.get("liftL");

        liftL.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()) {
            controller.setPID(p, i, d);
            int position = liftL.getCurrentPosition();
            double power = controller.calculate(position, target) + fG;
            liftR.setPower(power);
            liftL.setPower(power);
            telemetry.addData("left: ", liftL.getCurrentPosition());
            telemetry.addData("right", liftR.getCurrentPosition());
        }
    }
}
