package frc.robot.math

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.radians
import kotlin.math.cos
import kotlin.math.sin

fun sin(x: Angle): Double =
    sin(x.inUnit(radians))

fun cos(x: Angle): Double =
    cos(x.inUnit(radians))