package frc.robot

object ID {
    // arm motors: proximal are spark maxes, distal is falcon
    const val arm_proximal_one = 12
    // new; no id
    const val arm_proximal_two = 23
    // original was 20. Some people say it's the original; idk
    // wired to 15 slot on REV can bus i think
    const val arm_distal = 7

    // arm encoders: both CANcoder IDs
    // NOTE: UNUSED as of now(looks like we're going encoder-less!)
    const val arm_jointa_encoder_proximal = 0
    const val arm_jointb_encoder_distal = 0

    // drivetrain motors: all are neos(can bus IDs)
    // verified
    const val drive_left1 = 7
    const val drive_left2 = 11
    const val drive_right1 = 15
    const val drive_right2 = 9

    // intake motors: all are neos
    // finalized!
    const val intake_left = 8
    const val intake_right = 4
}