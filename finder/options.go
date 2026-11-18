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

// Option configures a finder.
type Option func(*FinderOptions)

// WithDiagonal sets the diagonal movement rule.
func WithDiagonal(d DiagonalMovement) Option {
	return func(o *FinderOptions) {
		o.DiagonalMovement = d
	}
}

// WithAllowDiagonal enables diagonal movement.
func WithAllowDiagonal(dontCrossCorners bool) Option {
	return func(o *FinderOptions) {
		o.AllowDiagonal = true
		o.DontCrossCorners = dontCrossCorners
	}
}

// WithWeight sets the heuristic weight.
func WithWeight(w float64) Option {
	return func(o *FinderOptions) {
		o.Weight = w
	}
}

// WithHeuristic sets the heuristic function.
func WithHeuristic(h HeuristicFunc) Option {
	return func(o *FinderOptions) {
		o.Heuristic = h
	}
}

// WithTrackRecursion enables recursion tracking.
func WithTrackRecursion() Option {
	return func(o *FinderOptions) {
		o.TrackRecursion = true
	}
}

// WithTimeLimit sets the time limit in seconds.
func WithTimeLimit(seconds float64) Option {
	return func(o *FinderOptions) {
		o.TimeLimit = seconds
	}
}

// ApplyOptions applies a list of Option functions to a FinderOptions.
func ApplyOptions(opts []Option) *FinderOptions {
	o := &FinderOptions{}
	for _, opt := range opts {
		opt(o)
	}
	return o
}

// resolveDiagonalMovement normalizes WithDiagonal / WithAllowDiagonal into a
// concrete rule, defaulting to DiagonalNever.
func resolveDiagonalMovement(o *FinderOptions) DiagonalMovement {
	if o.DiagonalMovement != 0 {
		return o.DiagonalMovement
	}
	if o.AllowDiagonal {
		if o.DontCrossCorners {
			return DiagonalOnlyWhenNoObstacles
		}
		return DiagonalIfAtMostOneObstacle
	}
	return DiagonalNever
}

// withDiagonal appends an explicit diagonal rule without mutating the caller's
// option slice.
func withDiagonal(opts []Option, d DiagonalMovement) []Option {
	out := make([]Option, 0, len(opts)+1)
	out = append(out, opts...)
	return append(out, WithDiagonal(d))
}
