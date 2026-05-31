package pathfinding

// Heuristic contains heuristic distance functions.
type Heuristic struct{}

var (
	Manhattan HeuristicFunc
	Euclidean HeuristicFunc
	Octile    HeuristicFunc
	Chebyshev HeuristicFunc
)

// HeuristicFunc is a function that computes distance given dx and dy.
type HeuristicFunc func(dx, dy float64) float64

func init() {
	Manhattan = func(dx, dy float64) float64 {
		return dx + dy
	}
	Euclidean = func(dx, dy float64) float64 {
		return sqrt(dx*dx + dy*dy)
	}
	Octile = func(dx, dy float64) float64 {
		f := SQRT2 - 1
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
