package pathfinding

// Grid represents the layout of nodes.
type Grid struct {
	Width  int
	Height int
	Nodes  [][]*Node
}

// NewGrid creates a new Grid with the given dimensions, optionally from a matrix.
// The matrix is a 2D slice where 0/false = walkable, non-zero/true = unwalkable.
func NewGrid(widthOrMatrix interface{}, args ...interface{}) *Grid {
	var width, height int
	var matrix [][]int

	switch v := widthOrMatrix.(type) {
	case int:
		width = v
		if len(args) > 0 {
			height = args[0].(int)
		}
		if len(args) > 1 {
			matrix = args[1].([][]int)
		}
	case [][]int:
		matrix = v
		height = len(matrix)
		if height > 0 {
			width = len(matrix[0])
		}
	}

	g := &Grid{
		Width:  width,
		Height: height,
	}
	g.buildNodes(matrix)
	return g
}

func (g *Grid) buildNodes(matrix [][]int) {
	nodes := make([][]*Node, g.Height)
	for y := 0; y < g.Height; y++ {
		nodes[y] = make([]*Node, g.Width)
		for x := 0; x < g.Width; x++ {
			nodes[y][x] = &Node{X: x, Y: y, Walkable: true}
		}
	}

	if matrix == nil {
		g.Nodes = nodes
		return
	}

	if len(matrix) != g.Height || len(matrix[0]) != g.Width {
		panic("Matrix size does not fit")
	}

	for y := 0; y < g.Height; y++ {
		for x := 0; x < g.Width; x++ {
			if matrix[y][x] != 0 {
				nodes[y][x].Walkable = false
			}
		}
	}
	g.Nodes = nodes
}

// GetNodeAt returns the node at (x, y).
func (g *Grid) GetNodeAt(x, y int) *Node {
	return g.Nodes[y][x]
}

// IsWalkableAt returns whether the node at (x, y) is walkable.
// Returns false if the position is outside the grid.
func (g *Grid) IsWalkableAt(x, y int) bool {
	return g.IsInside(x, y) && g.Nodes[y][x].Walkable
}

// IsInside returns whether (x, y) is inside the grid.
func (g *Grid) IsInside(x, y int) bool {
	return x >= 0 && x < g.Width && y >= 0 && y < g.Height
}

// SetWalkableAt sets the walkable status of the node at (x, y).
func (g *Grid) SetWalkableAt(x, y int, walkable bool) {
	g.Nodes[y][x].Walkable = walkable
}

// GetNeighbors returns the neighbors of the given node.
func (g *Grid) GetNeighbors(node *Node, diagonalMovement DiagonalMovement) []*Node {
	x, y := node.X, node.Y
	neighbors := make([]*Node, 0, 8)
	nodes := g.Nodes

	s0, d0 := false, false
	s1, d1 := false, false
	s2, d2 := false, false
	s3, d3 := false, false

	// up
	if g.IsWalkableAt(x, y-1) {
		neighbors = append(neighbors, nodes[y-1][x])
		s0 = true
	}
	// right
	if g.IsWalkableAt(x+1, y) {
		neighbors = append(neighbors, nodes[y][x+1])
		s1 = true
	}
	// down
	if g.IsWalkableAt(x, y+1) {
		neighbors = append(neighbors, nodes[y+1][x])
		s2 = true
	}
	// left
	if g.IsWalkableAt(x-1, y) {
		neighbors = append(neighbors, nodes[y][x-1])
		s3 = true
	}

	if diagonalMovement == DiagonalNever {
		return neighbors
	}

	switch diagonalMovement {
	case DiagonalOnlyWhenNoObstacles:
		d0 = s3 && s0
		d1 = s0 && s1
		d2 = s1 && s2
		d3 = s2 && s3
	case DiagonalIfAtMostOneObstacle:
		d0 = s3 || s0
		d1 = s0 || s1
		d2 = s1 || s2
		d3 = s2 || s3
	case DiagonalAlways:
		d0, d1, d2, d3 = true, true, true, true
	}

	// northwest
	if d0 && g.IsWalkableAt(x-1, y-1) {
		neighbors = append(neighbors, nodes[y-1][x-1])
	}
	// northeast
	if d1 && g.IsWalkableAt(x+1, y-1) {
		neighbors = append(neighbors, nodes[y-1][x+1])
	}
	// southeast
	if d2 && g.IsWalkableAt(x+1, y+1) {
		neighbors = append(neighbors, nodes[y+1][x+1])
	}
	// southwest
	if d3 && g.IsWalkableAt(x-1, y+1) {
		neighbors = append(neighbors, nodes[y+1][x-1])
	}

	return neighbors
}

// Clone creates a deep copy of the grid.
func (g *Grid) Clone() *Grid {
	newGrid := &Grid{
		Width:  g.Width,
		Height: g.Height,
	}
	newNodes := make([][]*Node, g.Height)
	for y := 0; y < g.Height; y++ {
		newNodes[y] = make([]*Node, g.Width)
		for x := 0; x < g.Width; x++ {
			newNodes[y][x] = &Node{
				X:        x,
				Y:        y,
				Walkable: g.Nodes[y][x].Walkable,
			}
		}
	}
	newGrid.Nodes = newNodes
	return newGrid
}
