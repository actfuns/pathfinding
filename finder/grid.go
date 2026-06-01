package finder

// Grid is the interface that all pathfinding algorithms depend on.
// Different grid layouts (orthogonal, hex, staggered) implement this interface
// with their own neighbor topology.
type Grid interface {
	Width() int
	Height() int
	IsInside(x, y int) bool
	IsWalkableAt(x, y int) bool
	SetWalkableAt(x, y int, walkable bool)
	GetNodeAt(x, y int) *Node
	GetNeighbors(node *Node, diagonal DiagonalMovement, buffer []*Node) []*Node
	Clone() Grid
}
