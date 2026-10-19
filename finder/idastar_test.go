package finder_test

import (
	"errors"
	"math/rand"
	"testing"
	"time"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func TestIDAStarAgainstJS(t *testing.T) {
	testFinderAgainstJS(t, "IDAStar", func() finder.Finder { return finder.NewIDAStarFinder() }, [][][2]int{
		{{0, 0}, {1, 0}, {1, 1}},
		{{1, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}},
		{{0, 3}, {0, 2}, {2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {3, 3}},
		{{4, 4}, {5, 4}, {6, 4}, {7, 4}, {8, 4}, {9, 4}, {10, 4}, {11, 4}, {12, 4}, {13, 4}, {14, 4}, {15, 4}, {16, 4}, {17, 4}, {18, 4}, {19, 4}, {19, 5}, {19, 6}, {19, 7}, {19, 8}, {19, 9}, {19, 10}, {19, 11}, {19, 12}, {19, 13}, {19, 14}, {19, 15}, {19, 16}, {19, 17}, {19, 18}, {19, 19}},
	})
}

// A TimeLimit that expires must end the search, not spin forever: the cutoff can
// no longer advance once every call bails at once, so the outer loop needs to be
// told the search gave up.
func TestIDATimeLimitGivesUp(t *testing.T) {
	rng := rand.New(rand.NewSource(3))
	m := make([][]int, 120)
	for y := range m {
		row := make([]int, 120)
		for x := range row {
			if rng.Float64() < 0.2 {
				row[x] = 1
			}
		}
		m[y] = row
	}
	m[0][0], m[119][119] = 0, 0
	g := grid.NewOrthogonalGrid(m)

	for _, limit := range []float64{0.001, 0.01} {
		f := finder.NewIDAStarFinder(finder.WithTimeLimit(limit))
		type result struct {
			path [][2]int
			err  error
		}
		out := make(chan result, 1)
		go func() {
			p, err := f.FindPath(0, 0, 119, 119, g)
			out <- result{p, err}
		}()

		select {
		case r := <-out:
			if !errors.Is(r.err, finder.ErrTimeLimit) {
				t.Fatalf("TimeLimit=%v: want ErrTimeLimit, got path len=%d err=%v",
					limit, len(r.path), r.err)
			}
			if r.path != nil {
				t.Errorf("TimeLimit=%v: timed-out search must not return a path", limit)
			}
		case <-time.After(5 * time.Second):
			t.Fatalf("TimeLimit=%v: FindPath never returned (infinite loop)", limit)
		}
	}
}

// Giving up is not the same as "no path": the caller must be able to tell them
// apart, or a busy map would permanently strand a unit that has a valid route.
func TestNoPathIsNotAnError(t *testing.T) {
	matrix := [][]int{
		{0, 0, 0},
		{0, 1, 0},
		{0, 0, 0},
	}
	g := grid.NewOrthogonalGrid(matrix)

	f := finder.NewAStarFinder()
	path, err := f.FindPath(0, 0, 2, 2, g)
	if err != nil {
		t.Fatalf("a failed search must report nil error, got %v", err)
	}
	if len(path) == 0 {
		t.Fatal("expected a path around the obstacle")
	}

	// Truly unreachable: nil path, nil error.
	blocked := [][]int{{0, 1, 0}, {0, 1, 0}, {0, 1, 0}}
	gb := grid.NewOrthogonalGrid(blocked)
	path, err = f.FindPath(0, 0, 2, 0, gb)
	if err != nil {
		t.Fatalf("unreachable target must be (nil, nil), got err=%v", err)
	}
	if path != nil {
		t.Fatalf("expected nil path for an unreachable target, got %v", path)
	}
}
