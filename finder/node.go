package finder

// Node represents a single node on a grid.
type Node struct {
	X        int
	Y        int
	Walkable bool

	// Pathfinding state (reused across searches)
	G, H, F float64
	Parent  *Node
	Opened  int // 0=unopened, 1=BY_START, 2=BY_END (for bidirectional)
	Closed  bool
	By      int // 1=BY_START, 2=BY_END (for bidirectional BFS)

	// JPF tracking
	Tested      bool
	RetainCount int

	// heap index
	HeapIndex int

	// SearchID matches the finder's search sequence to avoid resetting per node.
	// 0 = initial state, updated on first use by each finder call.
	SearchID int
}

// NewNode creates a node.
func NewNode(x, y int) *Node {
	return &Node{X: x, Y: y, Walkable: true}
}

// ResetSearch resets all search-related fields if seq doesn't match.
// This allows repeated use of the same node across multiple FindPath calls
// without cloning the entire grid.
func (n *Node) ResetSearch(seq int) {
	if n.SearchID != seq {
		n.SearchID = seq
		n.G = 0
		n.H = 0
		n.F = 0
		n.Parent = nil
		n.Opened = 0
		n.Closed = false
		n.By = 0
		n.Tested = false
		n.RetainCount = 0
		n.HeapIndex = 0
	}
}
