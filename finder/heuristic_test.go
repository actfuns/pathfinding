package finder_test

import (
	"reflect"
	"testing"

	"github.com/actfuns/pathfinding/finder"
)

func sameFunc(a, b finder.HeuristicFunc) bool {
	return reflect.ValueOf(a).Pointer() == reflect.ValueOf(b).Pointer()
}

// A caller-supplied heuristic must survive enabling diagonals; only the
// implicit default is upgraded from Manhattan to Octile.
func TestCustomHeuristicNotOverridden(t *testing.T) {
	opts := []finder.Option{
		finder.WithAllowDiagonal(false),
		finder.WithHeuristic(finder.Euclidean),
	}
	cases := map[string]finder.HeuristicFunc{
		"AStar":   finder.NewAStarFinder(opts...).Heuristic,
		"BiAStar": finder.NewBiAStarFinder(opts...).Heuristic,
		"IDAStar": finder.NewIDAStarFinder(opts...).Heuristic,
	}
	for name, got := range cases {
		if !sameFunc(got, finder.Euclidean) {
			t.Errorf("%s: custom heuristic was overridden", name)
		}
	}
}

func TestDiagonalDefaultsToOctile(t *testing.T) {
	f := finder.NewAStarFinder(finder.WithAllowDiagonal(false))
	if !sameFunc(f.Heuristic, finder.Octile) {
		t.Error("AStar with diagonals should default to Octile")
	}
}
