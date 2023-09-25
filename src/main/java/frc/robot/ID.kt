package frc.robot

object ID {
    // arm motors: proximal are spark maxes, distal is falcon
    const val arm_proximal_one = 12
    // new; no id
    const val arm_proximal_two = 23
    // original was 20. Some people say it's the original; idk
    // wired to 15 slot on REV can bus i think
    const val arm_distal = 7

    // drivetrain motors: all are neos(can bus IDs)
    // verified
    const val drive_left1 = 9
    const val drive_left2 = 15
    const val drive_right1 = 7
    const val drive_right2 = 11

    // intake motors: all are neos
    // finalized!
    const val intake_left = 8
    const val intake_right = 4
}