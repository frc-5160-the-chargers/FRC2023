package frc.robot.math

import com.batterystaple.kmeasure.quantities.Energy
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.rotations
import kotlin.math.E
import kotlin.math.sqrt

typealias Torque = Energy

val squedalians = 0.5.rotations / sqrt(E)
val Int.squedalians get() = ofUnit(frc.robot.math.squedalians)
val Double.squedalians get() = ofUnit(frc.robot.math.squedalians)
val Long.squedalians get() = ofUnit(frc.robot.math.squedalians)