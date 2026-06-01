package finder

import "math"

// HeuristicFunc is a function that computes distance given dx and dy.
type HeuristicFunc func(dx, dy float64) float64

var (
	Manhattan HeuristicFunc
	Euclidean HeuristicFunc
	Octile    HeuristicFunc
	Chebyshev HeuristicFunc
)

func init() {
	Manhattan = func(dx, dy float64) float64 {
		return dx + dy
	}
	Euclidean = func(dx, dy float64) float64 {
		return math.Sqrt(dx*dx + dy*dy)
	}
	Octile = func(dx, dy float64) float64 {
		f := math.Sqrt2 - 1
		if dx < dy {
			return f*dx + dy
		}
		return f*dy + dx
	}
	Chebyshev = func(dx, dy float64) float64 {
		if dx > dy {
			return dx
		}
		return dy
	}
}
