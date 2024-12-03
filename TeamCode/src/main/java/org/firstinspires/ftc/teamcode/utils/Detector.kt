package org.firstinspires.ftc.teamcode.utils

class Detector {
    private var current = false
    private var previous = false

    fun update(value: Boolean)  {
        previous=current
        current=value
    }
    fun risingEdge() = current && !previous
    fun fallingEdge() = !current && previous
}