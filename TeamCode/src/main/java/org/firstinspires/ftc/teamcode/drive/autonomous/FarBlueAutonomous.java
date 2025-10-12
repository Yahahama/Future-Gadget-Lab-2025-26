package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "FAR_BLUE_AUTONOMOUS", group = "Autonomous")
public class FarBlueAutonomous extends Autonomous {
    @Override
    public void runOpMode() {
        Positions.START startPos = Positions.START.BLUE_FAR;
        super.runOpMode();
    }
}
