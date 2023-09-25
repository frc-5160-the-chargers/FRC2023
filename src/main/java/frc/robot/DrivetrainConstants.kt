package frc.robot

import com.batterystaple.kmeasure.units.inches
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax

object DriveMotors{
    val left1  = neoSparkMax(canBusId = 9){
        inverted = false
    }
    val left2  = neoSparkMax(canBusId = 15){
        inverted = false
    }
    val right1 = neoSparkMax(canBusId = 7){
        inverted = true
    }
    val right2 = neoSparkMax(canBusId = 11){
        inverted = true
    }
}

object DrivetrainConstants{
    const val gearRatio = 1.0/10.71
    val wheelDiameter = 6.inches
    val width = 27.inches
}
