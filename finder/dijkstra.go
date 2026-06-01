package finder

// DijkstraFinder is a Dijkstra pathfinder (A* with heuristic = 0).
type DijkstraFinder struct {
	AStarFinder
}

// NewDijkstraFinder creates a new DijkstraFinder.
func NewDijkstraFinder(opt *FinderOptions) *DijkstraFinder {
	f := &DijkstraFinder{}
	inner := NewAStarFinder(opt)
	f.AStarFinder = *inner
	f.Heuristic = func(dx, dy float64) float64 {
		return 0
	}
	return f
}
