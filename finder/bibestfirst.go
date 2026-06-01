package finder

// BiBestFirstFinder is a bidirectional Best-First-Search pathfinder.
type BiBestFirstFinder struct {
	BiAStarFinder
}

// NewBiBestFirstFinder creates a new BiBestFirstFinder.
func NewBiBestFirstFinder(opts ...Option) *BiBestFirstFinder {
	f := &BiBestFirstFinder{}
	inner := NewBiAStarFinder(opts...)
	f.BiAStarFinder = *inner
	orig := f.Heuristic
	f.Heuristic = func(dx, dy float64) float64 {
		return orig(dx, dy) * 1000000
	}
	return f
}
