package pathfinding

// Node represents a single node on a grid.
type Node struct {
	X        int
	Y        int
	Walkable bool

	// Pathfinding state (reused across searches)
	g, h, f    float64
	parent     *Node
	opened     int // 0=unopened, 1=BY_START, 2=BY_END (for bidirectional)
	closed     bool
	by         int // 1=BY_START, 2=BY_END (for bidirectional BFS)

	// JPF tracking
	tested      bool
	retainCount int

	// heap index
	heapIndex int
}
