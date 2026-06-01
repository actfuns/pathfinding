package finder

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
