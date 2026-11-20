package finder

import (
	"math/rand"
	"testing"
)

// flatGrid is a minimal all-walkable Grid used to inspect precomputed jump
// distances without importing the grid package (which imports finder).
type flatGrid struct {
	w, h  int
	nodes []Node
}

func newFlatGrid(w, h int) *flatGrid {
	g := &flatGrid{w: w, h: h, nodes: make([]Node, w*h)}
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			g.nodes[y*w+x] = Node{X: int32(x), Y: int32(y), Walkable: true, Weight: 1}
		}
	}
	return g
}

func (g *flatGrid) Width() int32  { return int32(g.w) }
func (g *flatGrid) Height() int32 { return int32(g.h) }
func (g *flatGrid) IsInside(x, y int32) bool {
	return x >= 0 && x < int32(g.w) && y >= 0 && y < int32(g.h)
}
func (g *flatGrid) IsWalkableAt(x, y int32) bool {
	return g.IsInside(x, y) && g.nodes[int(y)*g.w+int(x)].Walkable
}
func (g *flatGrid) SetWalkableAt(x, y int32, v bool)  { g.nodes[int(y)*g.w+int(x)].Walkable = v }
func (g *flatGrid) GetNodeAt(x, y int32) *Node        { return &g.nodes[int(y)*g.w+int(x)] }
func (g *flatGrid) SupportsJPSCanonicalPruning() bool { return true }
func (g *flatGrid) Clone() Grid {
	ng := &flatGrid{w: g.w, h: g.h, nodes: make([]Node, len(g.nodes))}
	copy(ng.nodes, g.nodes)
	return ng
}
func (g *flatGrid) GetNeighbors(node *Node, _ DiagonalMovement, buf []*Node) []*Node {
	out := buf[:0]
	for _, d := range [][2]int32{{0, -1}, {1, 0}, {0, 1}, {-1, 0}} {
		x, y := node.X+d[0], node.Y+d[1]
		if g.IsWalkableAt(x, y) {
			out = append(out, g.GetNodeAt(x, y))
		}
	}
	return out
}

// The jump table uses int8 for maps up to 128 cells and int16 above that,
// so small maps keep the compact format while large maps cannot overflow.
func TestJPSPlusDynamicTableWidth(t *testing.T) {
	cases := []struct {
		name string
		w    int
		wide bool
		want int16 // east distance from (0,0)
	}{
		{"int8-small", 10, false, -9},
		{"int8-boundary", 128, false, -127},
		{"int16-boundary", 129, true, -128},
		{"int16-wide", 300, true, -299},
	}

	for _, tc := range cases {
		g := newFlatGrid(tc.w, 1)
		f := NewJPSPlusFinder(WithDiagonal(DiagonalNever))
		_ = f.Precompute(g)

		if f.wide != tc.wide {
			t.Errorf("%s: wide=%v, want %v", tc.name, f.wide, tc.wide)
		}
		if got := f.getDist(0, 0, dirE); got != tc.want {
			t.Errorf("%s: getDist(0,0,E)=%d, want %d", tc.name, got, tc.want)
		}

		cellBytes := 1
		if tc.wide {
			cellBytes = 2
		}
		if want := 12 + tc.w*1*8*cellBytes; f.PrecomputedSize() != want {
			t.Errorf("%s: PrecomputedSize()=%d, want %d", tc.name, f.PrecomputedSize(), want)
		}

		data, err := f.PrecomputedData()
		if err != nil {
			t.Fatalf("%s: PrecomputedData: %v", tc.name, err)
		}
		f2 := NewJPSPlusFinder(WithDiagonal(DiagonalNever))
		if err := f2.LoadPrecomputed(data); err != nil {
			t.Fatalf("%s: LoadPrecomputed: %v", tc.name, err)
		}
		if f2.wide != tc.wide {
			t.Errorf("%s: round-trip wide=%v, want %v", tc.name, f2.wide, tc.wide)
		}
		if got := f2.getDist(0, 0, dirE); got != tc.want {
			t.Errorf("%s: round-trip getDist=%d, want %d", tc.name, got, tc.want)
		}
	}
}

// At the int8 boundary (128) every table entry must be identical to the int16
// encoding; otherwise the cast silently wrapped.
func TestJPSPlusInt8MatchesInt16(t *testing.T) {
	rng := rand.New(rand.NewSource(7))
	const n = 128
	g := newFlatGrid(n, n)
	for y := 0; y < n; y++ {
		for x := 0; x < n; x++ {
			if rng.Float64() < 0.25 {
				g.SetWalkableAt(int32(x), int32(y), false)
			}
		}
	}

	narrow := NewJPSPlusFinder(WithDiagonal(DiagonalOnlyWhenNoObstacles))
	_ = narrow.Precompute(g)
	if narrow.wide {
		t.Fatal("128x128 grid should use int8")
	}
	wide := NewJPSPlusFinder(WithDiagonal(DiagonalOnlyWhenNoObstacles))
	wide.precompute(g, true)
	if !wide.wide {
		t.Fatal("forced int16 table was not built")
	}

	if len(narrow.jump8) != len(wide.jump16) {
		t.Fatalf("table lengths differ: %d vs %d", len(narrow.jump8), len(wide.jump16))
	}
	for i, v := range narrow.jump8 {
		if int16(v) != wide.jump16[i] {
			cell := i / 8
			dir := i % 8
			t.Fatalf("truncation at (%d,%d) dir=%d: int8=%d int16=%d",
				cell%n, cell/n, dir, v, wide.jump16[i])
		}
	}
}

// Both serialized widths must round-trip the full table.
func TestJPSPlusTableRoundTripBothWidths(t *testing.T) {
	cases := []struct {
		w, h int
		wide bool
	}{
		{64, 64, false},
		{129, 32, true},
	}
	rng := rand.New(rand.NewSource(11))
	for _, tc := range cases {
		g := newFlatGrid(tc.w, tc.h)
		for y := 0; y < tc.h; y++ {
			for x := 0; x < tc.w; x++ {
				if rng.Float64() < 0.2 {
					g.SetWalkableAt(int32(x), int32(y), false)
				}
			}
		}
		f := NewJPSPlusFinder(WithDiagonal(DiagonalOnlyWhenNoObstacles))
		_ = f.Precompute(g)
		if f.wide != tc.wide {
			t.Fatalf("%dx%d: wide=%v, want %v", tc.w, tc.h, f.wide, tc.wide)
		}
		data, err := f.PrecomputedData()
		if err != nil {
			t.Fatalf("%dx%d: %v", tc.w, tc.h, err)
		}
		f2 := NewJPSPlusFinder(WithDiagonal(DiagonalOnlyWhenNoObstacles))
		if err := f2.LoadPrecomputed(data); err != nil {
			t.Fatalf("%dx%d: LoadPrecomputed: %v", tc.w, tc.h, err)
		}
		if f2.wide != tc.wide || f2.width != int32(tc.w) || f2.height != int32(tc.h) {
			t.Fatalf("%dx%d: loaded wide=%v size=%dx%d", tc.w, tc.h, f2.wide, f2.width, f2.height)
		}
		if tc.wide {
			for i := range f.jump16 {
				if f.jump16[i] != f2.jump16[i] {
					t.Fatalf("%dx%d: int16 mismatch at %d", tc.w, tc.h, i)
				}
			}
		} else {
			for i := range f.jump8 {
				if f.jump8[i] != f2.jump8[i] {
					t.Fatalf("%dx%d: int8 mismatch at %d", tc.w, tc.h, i)
				}
			}
		}
	}
}
