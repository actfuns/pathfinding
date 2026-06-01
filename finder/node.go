package finder

// Node represents a single node on a grid.
type Node struct {
	X        int
	Y        int
	Walkable bool

	// Pathfinding state (reused across searches)
	G, H, F    float64
	Parent     *Node
	Opened     int // 0=unopened, 1=BY_START, 2=BY_END (for bidirectional)
	Closed     bool
	By         int // 1=BY_START, 2=BY_END (for bidirectional BFS)

	// JPF tracking
	Tested      bool
	RetainCount int

	// heap index
	HeapIndex int
}

// NewNode creates a node.
func NewNode(x, y int) *Node {
	return &Node{X: x, Y: y, Walkable: true}
}
