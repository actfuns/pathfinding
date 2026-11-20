package hpa

import (
	"errors"
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

// A compressed portal can border many runs on the opposite chunk edge; the
// dynamic slice must keep all of them.
func TestHPAExternalPortalCapacity(t *testing.T) {
	const cs = 16
	g := grid.NewOrthogonalGrid(grid.NewMatrix(2*cs, cs))
	// West edge of the right chunk: alternating walkable rows, creating 8
	// separate portals there. The left chunk's east edge is one long portal.
	for y := 1; y < cs; y += 2 {
		g.SetWalkableAt(cs, int32(y), false)
	}

	w := BuildWorld(g, cs)
	p := w.Portals[PortalKey(0, 0, DirE, cs)]
	if p == nil {
		t.Fatal("east portal not found")
	}
	if p.ExternalCount != 8 {
		t.Errorf("external connections = %d, want 8", p.ExternalCount)
	}
	if len(p.ExternalPortals) != int(p.ExternalCount) || len(p.ExternalCosts) != int(p.ExternalCount) {
		t.Errorf("slice lengths: portals=%d costs=%d count=%d",
			len(p.ExternalPortals), len(p.ExternalCosts), p.ExternalCount)
	}
}

// With diagonal movement, HPA must use the same movement rule for portal
// connectivity instead of hardcoding DiagonalNever.
func TestHPAUsesDiagonalConnectivity(t *testing.T) {
	const cs = 8
	m := make([][]int32, cs)
	for y := range m {
		m[y] = make([]int32, 2*cs)
		for x := range m[y] {
			m[y][x] = 1
		}
	}
	// Left chunk corridor on row 4, right chunk corridor on row 3, meeting at
	// the chunk border. The only crossing is the diagonal (7,4) <-> (8,3).
	for x := 0; x < cs; x++ {
		m[4][x] = 0
		m[3][cs+x] = 0
	}
	g := grid.NewOrthogonalGrid(m)

	never := NewHPAFinder(WithChunkSize(cs))
	never.Build(g)
	if p, err := never.FindPath(0, 4, 2*cs-1, 3, g, nil); err != nil {
		t.Fatalf("Never: %v", err)
	} else if p != nil {
		t.Fatalf("Never should have no route, got %v", p)
	}

	always := NewHPAFinder(WithChunkSize(cs), WithDiagonal(finder.DiagonalAlways))
	always.Build(g)
	p, err := always.FindPath(0, 4, 2*cs-1, 3, g, nil)
	if err != nil {
		t.Fatalf("Always: %v", err)
	}
	if len(p) == 0 {
		t.Fatal("Always should cross the diagonal-only boundary")
	}
	if p[0] != [2]int32{0, 4} || p[len(p)-1] != [2]int32{2*cs - 1, 3} {
		t.Fatalf("bad endpoints: %v .. %v", p[0], p[len(p)-1])
	}
	for _, q := range p {
		if !g.IsWalkableAt(q[0], q[1]) {
			t.Fatalf("path crosses wall at %v", q)
		}
	}
}

func TestHPABuildRejectsNonOrthogonal(t *testing.T) {
	hex := grid.NewHexGrid(grid.NewMatrix(8, 8))
	stag := grid.NewStaggeredGrid(grid.NewMatrix(8, 8))
	for name, g := range map[string]finder.Grid{"hex": hex, "staggered": stag} {
		err := NewHPAFinder().Build(g)
		if !errors.Is(err, finder.ErrUnsupportedGrid) {
			t.Errorf("%s: Build error = %v, want ErrUnsupportedGrid", name, err)
		}
	}
}
