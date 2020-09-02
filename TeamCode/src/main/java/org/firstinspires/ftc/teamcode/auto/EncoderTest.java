package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by tarunsingh on 12/5/17.
 */

@Autonomous(name="Encoder Test")
public class EncoderTest extends LinearOpMode {
    private static final int targetPosition = 315;
    private static final double maxPower = 0;
    private Robot robot;

    public void initOpMode(){
        ElapsedTime timer = new ElapsedTime();
        robot = new Robot(this, timer);
        robot.xRailWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.xRailWinch.setTargetPosition(0);
        robot.xRailWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.xRailWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.xRailWinch.setTargetPosition(0);
        robot.xRailWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("OK!", 5);
    }
    public void runOpMode() {
        initOpMode();
        waitForStart();
        robot.timer.reset();


//
        while (opModeIsActive()) {
            telemetry.addData("tilt", robot.armTilt.getCurrentPosition());
            telemetry.update();
        }
//        robot.xRailWinch.setPower(0.5);
//        robot.xRailWinch.setTargetPosition(8400);
//        while(robot.xRailWinch.isBusy()){
//            telemetry.addData("Winch", robot.xRailWinch.getCurrentPosition());
//            telemetry.update();
//        }
//        telemetry.addData("Winch", robot.xRailWinch.getCurrentPosition());
//        telemetry.update();


        sleep(10000);
    }
}