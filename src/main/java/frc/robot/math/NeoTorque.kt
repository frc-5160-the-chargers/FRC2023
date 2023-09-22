package frc.robot.math

import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.newtons
import frc.chargers.hardware.motorcontrol.rev.ChargerCANSparkMax


fun ChargerCANSparkMax.setStallTorqueNEO(torque: Torque) {
    setVoltage(
        stallTorqueNmToVoltage(torque.inUnit(newtons * meters))
    )
}

@Suppress("MagicNumber")
val stallTorqueNmToVoltage = Polynomial( // Determined empirically; polynomial fit is arbitrary
    0.0, 6.20254, -4.08666, 1.33192, -0.0981478
)