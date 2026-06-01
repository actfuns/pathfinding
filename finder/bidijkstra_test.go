package finder_test

import (
	"testing"

	"github.com/actfuns/navpath/finder"
)

func TestBiDijkstraAgainstJS(t *testing.T) {
	testFinderAgainstJS(t, "BiDijkstra", func() finder.Finder { return finder.NewBiDijkstraFinder() }, [][][2]int{
		{{0, 0}, {1, 0}, {1, 1}},
		{{1, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}},
		{{0, 3}, {1, 3}, {1, 4}, {1, 5}, {2, 5}, {3, 5}, {4, 5}, {4, 4}, {4, 3}, {3, 3}},
		{{4, 4}, {5, 4}, {6, 4}, {6, 5}, {7, 5}, {7, 6}, {7, 7}, {7, 8}, {7, 9}, {8, 9}, {8, 10}, {9, 10}, {9, 11}, {9, 12}, {10, 12}, {11, 12}, {11, 13}, {11, 14}, {12, 14}, {13, 14}, {14, 14}, {15, 14}, {16, 14}, {17, 14}, {17, 15}, {17, 16}, {18, 16}, {19, 16}, {19, 17}, {19, 18}, {19, 19}},
	})
}
