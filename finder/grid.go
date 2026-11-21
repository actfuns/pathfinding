package finder

// Grid is the interface that all pathfinding algorithms depend on.
// Different grid layouts (orthogonal, hex, staggered) implement this interface
// with their own neighbor topology.
type Grid interface {
	// Width returns the number of tiles horizontally in the grid.
	Width() int32
	// Height returns the number of tiles vertically in the grid.
	Height() int32
	// IsInside reports whether the tile (x, y) is within the grid bounds.
	IsInside(x, y int32) bool
	// IsWalkableAt reports whether the tile (x, y) is walkable.
	// Tiles outside the grid are reported as not walkable.
	IsWalkableAt(x, y int32) bool
	// GetNodeAt returns the node at tile (x, y). Panics if outside the grid.
	// The returned Node is owned by the grid; callers must not mutate it.
	GetNodeAt(x, y int32) *Node
	// GetNeighbors returns the walkable, adjacent neighbors of the given node.
	GetNeighbors(node *Node, diagonal DiagonalMovement, buffer []*Node) []*Node
}

// StepCostGrid is optionally implemented by grids whose step costs differ from
// the square-grid default (cardinal = 1, diagonal = sqrt(2)).
type StepCostGrid interface {
	StepCost(dx, dy int32) float32
}

// SymmetricNeighborGrid opts into bidirectional search. For each supported
// movement rule, a walkable tile must list another walkable tile as a neighbor
// exactly when the reverse edge exists. Edge costs may differ by direction.
type SymmetricNeighborGrid interface {
	Grid
	HasSymmetricNeighbors() bool
}

// heuristicGrid is optionally implemented by grids that need a topology-specific
// default heuristic (for example, hex grids with unit-cost moves).
type heuristicGrid interface {
	DefaultHeuristic(diagonal DiagonalMovement) HeuristicFunc
}

// gridStepCost returns the grid's step-cost function, or nil when the
// square-grid default applies. Returning nil keeps the inline fast path.
func gridStepCost(grid Grid) func(dx, dy int32) float32 {
	if g, ok := grid.(StepCostGrid); ok {
		return g.StepCost
	}
	return nil
}

// moveCost prices a directed edge using the forward destination's weight.
func moveCost(stepCost func(int32, int32) float32, dx, dy int32, tileWeight float32) float32 {
	if stepCost != nil {
		return stepCost(dx, dy) * tileWeight
	}
	if dx != 0 && dy != 0 {
		return SQRT2 * tileWeight
	}
	return tileWeight
}

// heuristicFor resolves the topology and terrain cost of the default
// heuristic. An explicit heuristic owns its cost model.
func heuristicFor(grid Grid, diagonal DiagonalMovement, heuristic HeuristicFunc, custom bool, weight float32) (HeuristicFunc, float32) {
	if custom {
		return heuristic, weight
	}
	if g, ok := grid.(heuristicGrid); ok {
		heuristic = g.DefaultHeuristic(diagonal)
	}
	if g, ok := grid.(interface{ HeuristicWeightLowerBound() float32 }); ok {
		weight *= g.HeuristicWeightLowerBound()
	}
	return heuristic, weight
}
