package grid

import "github.com/actfuns/pathfinding/finder"

func cloneSnapshot[T MutableGrid](source finder.Finder, copyGrid func() T, fn func(MutableGrid)) (Grid, error) {
	if fn == nil {
		return nil, ErrCloneFuncRequired
	}

	snapshot := copyGrid()
	if !finder.IsStateless(source) {
		snapshot.SetFinder(nil)
	}
	fn(snapshot)
	if snapshot.Finder() == nil {
		return nil, ErrFinderRequired
	}
	return snapshot, nil
}
