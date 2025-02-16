package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class OptimizedOpMode extends OpMode {
    @Override
    public abstract void init();

    @Override
    public abstract void loop();

    public abstract void keybinds();

    public void gamepad1Keybinds(){}

    public void gamepad2Keybinds(){}

}
