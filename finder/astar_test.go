package finder_test

import (
	"testing"

	"github.com/actfuns/navpath/finder"
	"github.com/actfuns/navpath/grid"
)

func TestAStarAgainstJS(t *testing.T) {
	testFinderAgainstJS(t, "AStar", func() finder.Finder { return finder.NewAStarFinder() }, [][][2]int{
		{{0, 0}, {1, 0}, {1, 1}},
		{{1, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}},
		{{0, 3}, {1, 3}, {1, 4}, {1, 5}, {2, 5}, {3, 5}, {4, 5}, {4, 4}, {4, 3}, {3, 3}},
		{{4, 4}, {5, 4}, {5, 5}, {6, 5}, {6, 6}, {6, 7}, {7, 7}, {7, 8}, {8, 8}, {9, 8}, {9, 9}, {10, 9}, {11, 9}, {12, 9}, {12, 10}, {12, 11}, {12, 12}, {12, 13}, {12, 14}, {13, 14}, {13, 15}, {14, 15}, {14, 16}, {14, 17}, {15, 17}, {16, 17}, {17, 17}, {18, 17}, {19, 17}, {19, 18}, {19, 19}},
	})
}

func TestAStarWithOptions(t *testing.T) {
	f := finder.NewAStarFinder(
		finder.WithAllowDiagonal(true),
		finder.WithWeight(2),
	)
	if f.Weight != 2 {
		t.Errorf("expected Weight=2, got %v", f.Weight)
	}
	if f.DiagonalMovement != finder.DiagonalOnlyWhenNoObstacles {
		t.Errorf("expected DiagonalOnlyWhenNoObstacles, got %v", f.DiagonalMovement)
	}

	f2 := finder.NewAStarFinder(finder.WithAllowDiagonal(false))
	if f2.DiagonalMovement != finder.DiagonalIfAtMostOneObstacle {
		t.Errorf("expected DiagonalIfAtMostOneObstacle, got %v", f2.DiagonalMovement)
	}

	f3 := finder.NewAStarFinder(
		finder.WithDiagonal(finder.DiagonalAlways),
		finder.WithHeuristic(finder.Euclidean),
	)
	if f3.DiagonalMovement != finder.DiagonalAlways {
		t.Errorf("expected DiagonalAlways, got %v", f3.DiagonalMovement)
	}
}

func TestAStarWeight(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	})
	f := finder.NewAStarFinder(finder.WithWeight(100))
	path := f.FindPath(0, 0, 4, 1, g)
	if path == nil {
		t.Error("expected path even with high weight")
	}
	if path[0] != [2]int{0, 0} || path[len(path)-1] != [2]int{4, 1} {
		t.Error("wrong start/end")
	}
}
