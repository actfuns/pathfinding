package finder

// DijkstraFinder is a Dijkstra pathfinder (A* with heuristic = 0).
type DijkstraFinder struct {
	AStarFinder
}

// NewDijkstraFinder creates a new DijkstraFinder.
func NewDijkstraFinder(opts ...Option) *DijkstraFinder {
	f := &DijkstraFinder{}
	inner := NewAStarFinder(opts...)
	f.AStarFinder = *inner
	f.Heuristic = func(dx, dy float32) float32 {
		return 0
	}
	f.customHeuristic = true
	return f
}
