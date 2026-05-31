package core

import (
	"math"
)

const SQRT2 = 1.4142135623730951

var Sqrt = math.Sqrt
var Abs = math.Abs

func AbsInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

func MaxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}
