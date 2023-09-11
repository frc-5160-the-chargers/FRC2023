# FRC2023
This is the repo for the 2023 year, including the offseason. 
The robot that this code is programmed for plays FRC's 2023 season; charged up.

Note: the offseason branch uses the experimental k2 compiler. There is a KNOWN ISSUE with the k2 compiler which would cause a java.util.ArrayIndexOutOfBoundsException IF you have a generic type that extends AnyDimension. 
If you have code that extends AnyDimension, comment out the k2 compiler dependency in gradle.properties.
