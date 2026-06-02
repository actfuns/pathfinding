package finder

// SQRT2 is the square root of 2, used as the cost of a diagonal step.
const SQRT2 = 1.4142135623730951

// AbsInt returns the absolute value of x.
func AbsInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// MaxInt returns the larger of a and b.
func MaxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}
