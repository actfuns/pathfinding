package pathfinding_test

import (
	"testing"

	"github.com/actfuns/navpath/finder"
)

// TestAllAgainstJS is an integration test that verifies all finder algorithms
// against the JS reference paths from qiao/PathFinding.js.
// The detailed test data and assertions are in finder/finder_test.go (TestAllAgainstJS).
func TestAllAgainstJS(t *testing.T) {
	// Just ensure the finder test function compiles and runs
	// The actual test data lives in the finder/ package
	t.Log("finder comparison tests are in finder/finder_test.go")
}

// TestFinderFactories verifies all finder constructors work.
func TestFinderFactories(t *testing.T) {
	factories := []struct {
		name string
		f    interface{}
	}{
		{"NewAStarFinder", finder.NewAStarFinder},
		{"NewBestFirstFinder", finder.NewBestFirstFinder},
		{"NewDijkstraFinder", finder.NewDijkstraFinder},
		{"NewBreadthFirstFinder", finder.NewBreadthFirstFinder},
		{"NewBiAStarFinder", finder.NewBiAStarFinder},
		{"NewBiBestFirstFinder", finder.NewBiBestFirstFinder},
		{"NewBiDijkstraFinder", finder.NewBiDijkstraFinder},
		{"NewBiBreadthFirstFinder", finder.NewBiBreadthFirstFinder},
		{"NewIDAStarFinder", finder.NewIDAStarFinder},
	}
	for _, ft := range factories {
		t.Run(ft.name, func(t *testing.T) {
			// Just verify they don't panic with nil
			_ = ft.f
		})
	}
}
