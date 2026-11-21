package grid

import (
	"errors"
	"testing"

	"github.com/actfuns/pathfinding/finder"
)

func TestMutableGridAssertionSharesStorage(t *testing.T) {
	g := NewOrthogonalGrid([][]int32{{0, 0}, {0, 0}})
	m, ok := g.(MutableGrid)
	if !ok {
		t.Fatal("built-in grid must implement MutableGrid")
	}
	m.SetWalkableAt(0, 0, false)
	if g.IsWalkableAt(0, 0) {
		t.Fatal("MutableGrid assertion must share the underlying storage")
	}
}

func TestMutableGridSetFinder(t *testing.T) {
	g := NewOrthogonalGrid([][]int32{{0, 0}, {0, 0}})
	f := finder.NewBreadthFirstFinder()
	m := g.(MutableGrid)
	m.SetFinder(f)
	if g.Finder() != f {
		t.Fatal("SetFinder did not replace the finder")
	}
}

func TestMutableGridWorldSetters(t *testing.T) {
	g := NewOrthogonalGrid([][]int32{{0, 0}, {0, 0}}, WithTileSize(10, 10))
	m := g.(MutableGrid)
	wx, wy := g.TileToWorld(1, 1)
	m.SetWalkableAtWorld(wx, wy, false)
	m.SetWeightAtWorld(wx, wy, 3)
	if g.IsWalkableAt(1, 1) {
		t.Error("SetWalkableAtWorld did not modify the tile")
	}
	if got := g.GetWeightAt(1, 1); got != 3 {
		t.Errorf("SetWeightAtWorld weight = %v, want 3", got)
	}
	if got := g.GetWeightAtWorld(wx, wy); got != 3 {
		t.Errorf("GetWeightAtWorld weight = %v, want 3", got)
	}
}

func TestCloneRebindsSmoother(t *testing.T) {
	g := NewOrthogonalGrid([][]int32{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	}, WithSmoothBresenham())
	g2, err := g.Clone(func(m MutableGrid) {
		m.SetWalkableAt(1, 1, false)
	})
	if err != nil {
		t.Fatalf("Clone: %v", err)
	}
	path, err := g2.FindPath(0, 1, 2, 1, nil)
	if err != nil {
		t.Fatalf("FindPath: %v", err)
	}
	for _, p := range path {
		if p == [2]int32{1, 1} {
			t.Fatalf("smoothed path crosses cloned wall: %v", path)
		}
	}
}

func TestCloneRequiresFinderForStateful(t *testing.T) {
	g := NewOrthogonalGrid([][]int32{{0, 0}, {0, 0}}, WithFinder(finder.NewJPSPlusFinder()))
	for _, fn := range []func(MutableGrid){nil, func(MutableGrid) {}} {
		if _, err := g.Clone(fn); !errors.Is(err, ErrFinderRequired) {
			t.Fatalf("Clone error = %v, want ErrFinderRequired", err)
		}
	}
}

func TestCloneAcceptsReplacementFinder(t *testing.T) {
	g := NewOrthogonalGrid([][]int32{{0, 0}, {0, 0}}, WithFinder(finder.NewJPSPlusFinder()))
	f := finder.NewBreadthFirstFinder()
	g2, err := g.Clone(func(m MutableGrid) {
		m.SetFinder(f)
	})
	if err != nil {
		t.Fatalf("Clone with replacement finder: %v", err)
	}
	if g2.Finder() != f {
		t.Fatal("Clone did not install the replacement finder")
	}
}

func TestCloneReturnsSnapshot(t *testing.T) {
	grids := []struct {
		name string
		g    Grid
	}{
		{"orthogonal", NewOrthogonalGrid([][]int32{{0, 0}, {0, 0}})},
		{"hex", NewHexGrid([][]int32{{0, 0}, {0, 0}})},
		{"staggered", NewStaggeredGrid([][]int32{{0, 0}, {0, 0}})},
	}
	for _, tc := range grids {
		t.Run(tc.name, func(t *testing.T) {
			snapshot, err := tc.g.Clone(nil)
			if err != nil || snapshot == nil {
				t.Fatalf("Clone(nil) = %v, %v, want an unedited snapshot", snapshot, err)
			}
			if !snapshot.IsWalkableAt(0, 0) || snapshot.GetWeightAt(1, 1) != 1 {
				t.Fatal("Clone(nil) changed the snapshot")
			}
			g2, err := tc.g.Clone(func(m MutableGrid) {
				m.SetWalkableAt(0, 0, false)
				m.SetWeightAt(1, 1, 5)
			})
			if err != nil {
				t.Fatalf("Clone: %v", err)
			}
			if g2 == nil {
				t.Fatal("Clone returned nil")
			}
			if !tc.g.IsWalkableAt(0, 0) {
				t.Error("Clone modified the original grid")
			}
			if tc.g.GetWeightAt(1, 1) != 1 {
				t.Error("Clone modified the original weight")
			}
			if g2.IsWalkableAt(0, 0) {
				t.Error("Clone did not apply walkability edit")
			}
			if got := g2.GetWeightAt(1, 1); got != 5 {
				t.Errorf("Clone weight = %v, want 5", got)
			}
		})
	}
}
