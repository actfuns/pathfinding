package finder

// BestFirstFinder is a Best-First-Search pathfinder.
// It extends AStar by using a very high heuristic weight (1,000,000x).
type BestFirstFinder struct {
	AStarFinder
}

// NewBestFirstFinder creates a new BestFirstFinder.
func NewBestFirstFinder(opts ...Option) *BestFirstFinder {
	f := &BestFirstFinder{}
	inner := NewAStarFinder(opts...)
	f.AStarFinder = *inner
	orig := f.Heuristic
	f.Heuristic = func(dx, dy float32) float32 {
		return orig(dx, dy) * 1000000
	}
	f.customHeuristic = true
	return f
}
