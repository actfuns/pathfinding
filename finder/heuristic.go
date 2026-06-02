package finder

import "math"

// HeuristicFunc is a function that computes distance given dx and dy.
type HeuristicFunc func(dx, dy float64) float64

var (
	// Manhattan is the Manhattan distance heuristic: dx + dy.
	// Suitable for grids with cardinal-only movement.
	Manhattan HeuristicFunc

	// Euclidean is the straight-line distance heuristic: sqrt(dx^2 + dy^2).
	// Suitable for grids with any-angle movement.
	Euclidean HeuristicFunc

	// Octile is the octile distance heuristic: (sqrt2-1)*min(dx,dy) + max(dx,dy).
	// Suitable for grids with diagonal movement at sqrt(2) cost.
	Octile HeuristicFunc

	// Chebyshev is the Chebyshev distance heuristic: max(dx, dy).
	// Suitable for grids where diagonal movement cost equals cardinal cost.
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
