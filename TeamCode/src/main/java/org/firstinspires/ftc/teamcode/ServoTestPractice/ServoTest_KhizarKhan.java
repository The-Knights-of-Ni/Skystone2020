package org.firstinspires.ftc.teamcode.ServoTestPractice;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;

import java.io.IOException;

@TeleOp(name="Servo Test KhizarK")
public class ServoTest_KhizarKhan extends LinearOpMode{
    private Robot robot;

    double PreTime = 0;
    double CurrentTime = 0;
    ElapsedTime timer = new ElapsedTime();

    public void initOpMode() throws IOException {
        this.robot = new Robot (this, timer);

    }

    public void runOpMode() {
        try {
            initOpMode();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        robot.initServosAuto();
        telemetry.clearAll();

        waitForStart(); //waits for start
        //leftstickx, leftstickx2, leftsticky, leftsticky2, etc.
        while (opModeIsActive()) {
            if (robot.mainClawArm )
        }





    }


}
