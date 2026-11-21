package finder

import (
	"bytes"
	"encoding/binary"
	"errors"
	"math/rand"
	"testing"
)

type jpsGridWithoutFingerprint struct {
	GridWithJPSSupport
	walkabilityChecks int
}

func (g *jpsGridWithoutFingerprint) IsWalkableAt(x, y int32) bool {
	g.walkabilityChecks++
	return g.GridWithJPSSupport.IsWalkableAt(x, y)
}

func TestJPSPlusGridWithoutFingerprintAvoidsFullScan(t *testing.T) {
	base := newFlatGrid(128, 128)
	g := &jpsGridWithoutFingerprint{GridWithJPSSupport: base}
	f := NewJPSPlusFinder(WithDiagonal(DiagonalAlways))
	if err := f.Precompute(g); err != nil {
		t.Fatal(err)
	}
	g.walkabilityChecks = 0
	base.SetWalkableAt(1, 0, false)
	base.SetWalkableAt(0, 1, false)
	base.SetWalkableAt(1, 1, false)
	path, err := f.FindPath(0, 0, 2, 0, g, nil)
	if err != nil || path != nil {
		t.Fatalf("got %v, %v, want no path from the isolated start", path, err)
	}
	if g.walkabilityChecks >= int(g.Width()*g.Height()) {
		t.Fatalf("query scanned the entire grid: %d walkability checks", g.walkabilityChecks)
	}
}

func TestJPSPlusRejectsOversizedTableBeforeScanning(t *testing.T) {
	// Deliberately omit node storage. Invalid dimensions must be rejected
	// before the grid is inspected or a jump table is allocated.
	g := &flatGrid{w: 32768, h: 8192}
	f := NewJPSPlusFinder(WithDiagonal(DiagonalAlways))
	if err := f.Precompute(g); !errors.Is(err, ErrUnsupportedGrid) {
		t.Fatalf("Precompute error = %v, want ErrUnsupportedGrid", err)
	}
}

func FuzzJPSPlusLoadPrecomputed(f *testing.F) {
	option := WithDiagonal(DiagonalAlways)
	original := NewJPSPlusFinder(option)
	narrowGrid := newFlatGrid(8, 3)
	for _, g := range []*flatGrid{narrowGrid, newFlatGrid(129, 3)} {
		if err := original.Precompute(g); err != nil {
			f.Fatal(err)
		}
		data, err := original.PrecomputedData()
		if err != nil {
			f.Fatal(err)
		}
		f.Add(data)

		withoutChecksum := bytes.Clone(data[:len(data)-4])
		binary.LittleEndian.PutUint32(withoutChecksum, binary.LittleEndian.Uint32(data)&^jpsPlusChecksumFlag)
		f.Add(withoutChecksum)

		legacy := append(bytes.Clone(withoutChecksum[:12]), withoutChecksum[20:]...)
		binary.LittleEndian.PutUint32(legacy, binary.LittleEndian.Uint32(data)&^(jpsPlusFingerprintFlag|jpsPlusChecksumFlag|jpsPlusCanonicalFlag))
		f.Add(legacy)
	}
	f.Add([]byte{})
	if err := original.Precompute(narrowGrid); err != nil {
		f.Fatal(err)
	}
	baseline, err := original.PrecomputedData()
	if err != nil {
		f.Fatal(err)
	}

	f.Fuzz(func(t *testing.T, input []byte) {
		finder := NewJPSPlusFinder(option)
		if err := finder.LoadPrecomputed(baseline); err != nil {
			t.Fatal(err)
		}
		err := finder.LoadPrecomputed(input)
		encoded, exportErr := finder.PrecomputedData()
		if exportErr != nil {
			t.Fatal(exportErr)
		}
		if err != nil {
			if !bytes.Equal(encoded, baseline) {
				t.Fatal("rejected input changed the active table")
			}
			return
		}

		reloaded := NewJPSPlusFinder(option)
		if err := reloaded.LoadPrecomputed(encoded); err != nil {
			t.Fatalf("could not load re-encoded table: %v", err)
		}
		roundTrip, err := reloaded.PrecomputedData()
		if err != nil || !bytes.Equal(roundTrip, encoded) {
			t.Fatalf("round trip changed the table: %v", err)
		}
	})
}

func TestJPSPlusLegacyTableUsesLiveGrid(t *testing.T) {
	g := newFlatGrid(4, 1)
	f := NewJPSPlusFinder(WithDiagonal(DiagonalAlways))
	if err := f.Precompute(g); err != nil {
		t.Fatal(err)
	}
	data, err := f.PrecomputedData()
	if err != nil {
		t.Fatal(err)
	}
	legacy := append(data[:12:12], data[20:len(data)-4]...)
	magic := binary.LittleEndian.Uint32(legacy)
	binary.LittleEndian.PutUint32(legacy, magic&^(jpsPlusFingerprintFlag|jpsPlusChecksumFlag))
	if err := f.LoadPrecomputed(legacy); err != nil {
		t.Fatal(err)
	}
	g.SetWalkableAt(2, 0, false)
	if path, err := f.FindPath(0, 0, 3, 0, g, nil); path != nil || err != nil {
		t.Fatalf("got %v, %v, want no path through the new obstacle", path, err)
	}
}

func TestJPSPlusRejectsCorruptTableWithoutReplacingIt(t *testing.T) {
	g := newFlatGrid(4, 2)
	f := NewJPSPlusFinder(WithDiagonal(DiagonalAlways))
	if err := f.Precompute(g); err != nil {
		t.Fatal(err)
	}
	data, err := f.PrecomputedData()
	if err != nil {
		t.Fatal(err)
	}
	bad := append([]byte(nil), data...)
	bad[20+dirE] = 127
	if err := f.LoadPrecomputed(bad); err == nil {
		t.Fatal("accepted corrupted checksum")
	}
	unchecked := bad[:len(bad)-4]
	binary.LittleEndian.PutUint32(unchecked, binary.LittleEndian.Uint32(unchecked)&^jpsPlusChecksumFlag)
	if err := f.LoadPrecomputed(unchecked); err == nil {
		t.Fatal("accepted an out-of-bounds distance without a checksum")
	}
	path, err := f.FindPath(0, 0, 3, 0, g, nil)
	if err != nil || len(path) != 4 {
		t.Fatalf("failed load replaced the existing table: %v, %v", path, err)
	}
}

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
		if want := 24 + tc.w*1*8*cellBytes; f.PrecomputedSize() != want {
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
