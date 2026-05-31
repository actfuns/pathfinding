package finder

// BiDijkstraFinder is a bidirectional Dijkstra pathfinder.
type BiDijkstraFinder struct {
	BiAStarFinder
}

// NewBiDijkstraFinder creates a new BiDijkstraFinder.
func NewBiDijkstraFinder(opt *FinderOptions) *BiDijkstraFinder {
	f := &BiDijkstraFinder{}
	inner := NewBiAStarFinder(opt)
	f.BiAStarFinder = *inner
	f.Heuristic = func(dx, dy float64) float64 {
		return 0
	}
	return f
}
