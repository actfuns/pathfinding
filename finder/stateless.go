package finder

// statelessFinder is an internal marker for finders that keep no per-grid
// precomputed state. Unknown or external finders are conservatively treated as
// stateful.
type statelessFinder interface {
	statelessFinder()
}

// IsStateless reports whether f is safe to reuse after a grid snapshot is
// cloned. Finders that cache topology do not implement the marker.
func IsStateless(f Finder) bool {
	_, ok := f.(statelessFinder)
	return ok
}
