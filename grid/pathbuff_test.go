package grid

import "testing"

func TestPathBuffReuse(t *testing.T) {
	matrix := NewMatrix(20, 20)
	for y := 0; y < 20; y++ {
		matrix[y][10] = 1
	}
	matrix[10][10] = 0
	g := NewOrthogonalGrid(matrix)
	buff := NewPathBuff(1, 1)

	p1, err := g.FindPathWorld(0, 0, 19, 19, buff)
	if err != nil || p1 == nil {
		t.Fatalf("first path: %v %v", p1, err)
	}
	if len(buff.Tile) != len(p1) {
		t.Fatalf("tile header not updated: %d vs %d", len(buff.Tile), len(p1))
	}
	if len(buff.World) != len(p1) {
		t.Fatalf("world header not updated: %d vs %d", len(buff.World), len(p1))
	}

	p2, err := g.FindPathWorld(19, 19, 0, 0, buff)
	if err != nil || p2 == nil {
		t.Fatalf("second path: %v %v", p2, err)
	}
	if len(p2) != len(p1) {
		t.Fatalf("second path length: %d vs %d", len(p2), len(p1))
	}

	buff.Reset()
	if len(buff.Tile) != 0 || len(buff.World) != 0 {
		t.Fatal("Reset must clear lengths")
	}
}
