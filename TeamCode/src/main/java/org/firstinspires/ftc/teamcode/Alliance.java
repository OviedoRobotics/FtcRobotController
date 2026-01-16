package org.firstinspires.ftc.teamcode;

public enum Alliance {
    RED,
    BLUE;

    double redOrBlue(double ifRed, double ifBlue) {
        return this == RED ? ifRed : ifBlue;
    }
}
