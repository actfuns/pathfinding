package pathfinding

// FinderOptions holds common options for all finders.
type FinderOptions struct {
	AllowDiagonal    bool
	DontCrossCorners bool
	DiagonalMovement DiagonalMovement
	Heuristic        HeuristicFunc
	Weight           float64
	TrackRecursion   bool
	TimeLimit        float64 // in seconds, <= 0 for infinite
}

// BestFirstFinder is a Best-First-Search pathfinder.
// It extends AStar by using a very high heuristic weight (1,000,000x).
type BestFirstFinder struct {
	AStarFinder
}

// NewBestFirstFinder creates a new BestFirstFinder.
func NewBestFirstFinder(opt *FinderOptions) *BestFirstFinder {
	f := &BestFirstFinder{}
	inner := NewAStarFinder(opt)
	f.AStarFinder = *inner
	// Override heuristic: multiply by 1,000,000
	orig := f.Heuristic
	f.Heuristic = func(dx, dy float64) float64 {
		return orig(dx, dy) * 1000000
	}
	return f
}
