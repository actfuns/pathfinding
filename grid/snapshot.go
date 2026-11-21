package grid

import "github.com/actfuns/pathfinding/finder"

func cloneSnapshot[T MutableGrid](source finder.Finder, copyGrid func() T, fn func(MutableGrid)) (Grid, error) {
	snapshot := copyGrid()
	if !finder.IsStateless(source) {
		snapshot.SetFinder(nil)
	}
	if fn != nil {
		fn(snapshot)
	}
	if snapshot.Finder() == nil {
		return nil, ErrFinderRequired
	}
	return snapshot, nil
}
