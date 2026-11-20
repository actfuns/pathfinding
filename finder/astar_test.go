package finder_test

import (
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func TestAStarAgainstJS(t *testing.T) {
	testFinderAgainstJS(t, "AStar", func() finder.Finder { return finder.NewAStarFinder() }, [][][2]int{
		{{0, 0}, {1, 0}, {1, 1}},
		{{1, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}},
		{{0, 3}, {1, 3}, {1, 4}, {1, 5}, {2, 5}, {3, 5}, {4, 5}, {4, 4}, {4, 3}, {3, 3}},
		{{4, 4}, {5, 4}, {6, 4}, {7, 4}, {8, 4}, {9, 4}, {10, 4}, {11, 4}, {12, 4}, {13, 4}, {14, 4}, {15, 4}, {16, 4}, {17, 4}, {18, 4}, {19, 4}, {19, 5}, {19, 6}, {19, 7}, {19, 8}, {19, 9}, {19, 10}, {19, 11}, {19, 12}, {19, 13}, {19, 14}, {19, 15}, {19, 16}, {19, 17}, {19, 18}, {19, 19}},
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
	path, _ := f.FindPath(0, 0, 4, 1, g, nil)
	if path == nil {
		t.Error("expected path even with high weight")
	}
	if path[0] != [2]int{0, 0} || path[len(path)-1] != [2]int{4, 1} {
		t.Error("wrong start/end")
	}
}
