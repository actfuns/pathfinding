package pathfinding

import (
	"math"
)

const SQRT2 = 1.4142135623730951

var sqrt = math.Sqrt
var abs = math.Abs

func absInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}
