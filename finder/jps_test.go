package finder_test

import (
	"testing"

	"github.com/actfuns/navpath/finder"
)

func TestJPFNeverAgainstJS(t *testing.T) {
	testFinderAgainstJS(t, "JPFNever", func() finder.Finder {
		return finder.JumpPointFinder(finder.WithDiagonal(finder.DiagonalNever))
	}, [][][2]int{
		{{0, 0}, {1, 0}, {1, 1}},
		{{1, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}},
		{{0, 3}, {1, 3}, {1, 4}, {1, 5}, {2, 5}, {3, 5}, {4, 5}, {4, 4}, {4, 3}, {3, 3}},
		{{4, 4}, {4, 5}, {4, 6}, {4, 7}, {4, 8}, {4, 9}, {4, 10}, {4, 11}, {4, 12}, {4, 13}, {4, 14}, {4, 15}, {4, 16}, {4, 17}, {4, 18}, {4, 19}, {5, 19}, {6, 19}, {7, 19}, {8, 19}, {9, 19}, {10, 19}, {11, 19}, {12, 19}, {13, 19}, {14, 19}, {15, 19}, {16, 19}, {17, 19}, {18, 19}, {19, 19}},
	})
}

func TestJPFAlwaysAgainstJS(t *testing.T) {
	testFinderAgainstJS(t, "JPFAlways", func() finder.Finder {
		return finder.JumpPointFinder(finder.WithDiagonal(finder.DiagonalAlways))
	}, [][][2]int{
		{{0, 0}, {1, 1}},
		{{1, 1}, {1, 2}, {2, 3}, {3, 3}, {4, 4}},
		{{0, 3}, {1, 4}, {2, 5}, {3, 5}, {4, 4}, {3, 3}},
		{{4, 4}, {5, 5}, {6, 6}, {7, 7}, {8, 8}, {9, 9}, {10, 10}, {11, 11}, {12, 12}, {13, 13}, {14, 14}, {15, 15}, {16, 16}, {17, 17}, {18, 18}, {19, 19}},
	})
}

func TestJPFOnlyWhenNoObstaclesAgainstJS(t *testing.T) {
	testFinderAgainstJS(t, "JPFOnlyWhenNoObstacles", func() finder.Finder {
		return finder.JumpPointFinder(finder.WithDiagonal(finder.DiagonalOnlyWhenNoObstacles))
	}, [][][2]int{
		{{0, 0}, {1, 0}, {1, 1}},
		{{1, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}},
		{{0, 3}, {1, 3}, {1, 4}, {1, 5}, {2, 5}, {3, 5}, {4, 5}, {4, 4}, {4, 3}, {3, 3}},
		{{4, 4}, {5, 5}, {6, 6}, {7, 7}, {8, 8}, {9, 9}, {10, 10}, {11, 11}, {12, 12}, {13, 13}, {14, 14}, {15, 15}, {16, 16}, {17, 17}, {18, 18}, {19, 19}},
	})
}

func TestJPFAtMostOneAgainstJS(t *testing.T) {
	testFinderAgainstJS(t, "JPFAtMostOne", func() finder.Finder {
		return finder.JumpPointFinder(finder.WithDiagonal(finder.DiagonalIfAtMostOneObstacle))
	}, [][][2]int{
		{{0, 0}, {1, 1}},
		{{1, 1}, {2, 0}, {3, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}},
		{{0, 3}, {1, 4}, {2, 5}, {3, 5}, {4, 4}, {3, 3}},
		{{4, 4}, {5, 5}, {6, 6}, {7, 7}, {8, 8}, {9, 9}, {10, 10}, {11, 11}, {12, 12}, {13, 13}, {14, 14}, {15, 15}, {16, 16}, {17, 17}, {18, 18}, {19, 19}},
	})
}
