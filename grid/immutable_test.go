package grid

import "testing"

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
}

func TestCloneWithRebindsSmoother(t *testing.T) {
	g := NewOrthogonalGrid([][]int32{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	}, WithSmoothBresenham())
	g2 := g.CloneWith(func(m MutableGrid) {
		m.SetWalkableAt(1, 1, false)
	})
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

func TestCloneWithReturnsSnapshot(t *testing.T) {
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
			g2 := tc.g.CloneWith(func(m MutableGrid) {
				m.SetWalkableAt(0, 0, false)
				m.SetWeightAt(1, 1, 5)
			})
			if g2 == nil {
				t.Fatal("CloneWith returned nil")
			}
			if !tc.g.IsWalkableAt(0, 0) {
				t.Error("CloneWith modified the original grid")
			}
			if tc.g.GetWeightAt(1, 1) != 1 {
				t.Error("CloneWith modified the original weight")
			}
			if g2.IsWalkableAt(0, 0) {
				t.Error("CloneWith did not apply walkability edit")
			}
			if got := g2.GetWeightAt(1, 1); got != 5 {
				t.Errorf("CloneWith weight = %v, want 5", got)
			}
		})
	}
}
