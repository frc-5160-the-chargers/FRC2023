package frc.robot.math.matrix
import org.ejml.equation.Operation
import org.ejml.equation.Sequence

operator fun Sequence.plus(other: Sequence): Sequence {
    if (this is MultipleSequence) {
        addSequence(other)
        return this
    } else {
        return MultipleSequence(this, other)
    }
}

class MultipleSequence(vararg sequences: Sequence) : Sequence() {
    private val sequences = mutableListOf(Sequence() /* Ensure at least one sequence in list */, *sequences)

    fun addSequence(sequence: Sequence) {
        sequences.add(sequence)
    }

    override fun addOperation(operation: Operation?) {
        sequences.last().addOperation(operation)
    }

    override fun perform() {
        sequences.forEach { it.perform() }
    }
}