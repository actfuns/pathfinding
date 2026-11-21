package finder

import "fmt"

// Direction index mapping for JPS+ jump table.
// Each cell stores 8 distances: N, NE, E, SE, S, SW, W, NW.
const (
	dirN  = 0
	dirNE = 1
	dirE  = 2
	dirSE = 3
	dirS  = 4
	dirSW = 5
	dirW  = 6
	dirNW = 7
)

var jpsCardinalDirs = [4][2]int32{
	{0, -1}, // N
	{1, 0},  // E
	{0, 1},  // S
	{-1, 0}, // W
}

var jpsDiagonalDirs = [4][2]int32{
	{1, -1},  // NE
	{1, 1},   // SE
	{-1, 1},  // SW
	{-1, -1}, // NW
}

// jpsCardinalIdx maps cardinal direction vectors to dir index.
var jpsCardinalIdx = map[[2]int32]int{
	{0, -1}: dirN,
	{1, 0}:  dirE,
	{0, 1}:  dirS,
	{-1, 0}: dirW,
}

// jpsDiagonalIdx maps diagonal direction vectors to dir index.
var jpsDiagonalIdx = map[[2]int32]int{
	{1, -1}:  dirNE,
	{1, 1}:   dirSE,
	{-1, 1}:  dirSW,
	{-1, -1}: dirNW,
}

// JPSPlusFinder implements Jump Point Search Plus — a precomputation-based
// optimization of JPS. It replaces the iterative jump() function with an
// O(1) table lookup of precomputed jump distances.
//
// Call Precompute(grid) before FindPath for axis-aligned grids. The grid is
// assumed to be static; changes to the grid require calling Precompute again.
// Precompute returns ErrUnsupportedGrid for hex/staggered grids; FindPath also
// rejects them.
//
// DiagonalAlways uses canonical JPS pruning with goal-dependent jumps resolved
// at query time. Other modes and legacy tables also relax one-step moves.
//
// JPS+ assumes every step has the same cost (tile Weight == 1). Tile weights are
// ignored; use A* or Dijkstra for weighted grids.
//
// JPS+ only supports axis-aligned grids (OrthogonalGrid). Hex and staggered
// grids return ErrUnsupportedGrid.
type JPSPlusFinder struct {
	Heuristic        HeuristicFunc
	DiagonalMovement DiagonalMovement
	Weight           float32
	Adaptive         bool

	jumpTable

	// Pre-allocated direction slice (avoid allocation in hot path)
	dirs []int
}

// NewJPSPlusFinder creates a JPS+ finder with the given options.
func NewJPSPlusFinder(opts ...Option) *JPSPlusFinder {
	opt := ApplyOptions(opts)
	diagonal := resolveDiagonalMovement(opt)
	var dirs []int
	if diagonal == DiagonalNever {
		dirs = []int{dirN, dirE, dirS, dirW}
	} else {
		dirs = []int{dirN, dirNE, dirE, dirSE, dirS, dirSW, dirW, dirNW}
	}
	f := &JPSPlusFinder{
		Heuristic:        Manhattan,
		DiagonalMovement: diagonal,
		Weight:           1,
		Adaptive:         opt.Adaptive,
		dirs:             dirs,
	}
	if opt.Heuristic != nil {
		f.Heuristic = opt.Heuristic
	} else if diagonal != DiagonalNever {
		f.Heuristic = Octile
	}
	if opt.Weight > 0 {
		f.Weight = opt.Weight
	}
	return f
}

// Precompute builds the jump distance table for the given grid.
// Must be called before FindPath. The grid is assumed static.
func (j *JPSPlusFinder) Precompute(grid Grid) error {
	if !supportsJPS(grid) {
		return ErrUnsupportedGrid
	}
	return j.precompute(grid, grid.Width() > 128 || grid.Height() > 128)
}

// precompute builds the table with an explicit cell width. It is split out so
// tests can compare the int8 and int16 encodings of the same grid.
func (j *JPSPlusFinder) precompute(grid Grid, wide bool) error {
	width, height := grid.Width(), grid.Height()
	size, ok := jumpTableSize(uint32(width), uint32(height), wide)
	if !ok {
		return fmt.Errorf("%w: JPS+ jump table dimensions %dx%d", ErrUnsupportedGrid, width, height)
	}
	j.width, j.height = width, height
	j.wide = wide
	j.fingerprint = gridWalkabilityFingerprint(grid)
	j.hasFingerprint = true
	j.canonical = j.DiagonalMovement == DiagonalAlways
	j.resize(size)
	clear(j.jump8)
	clear(j.jump16)

	if j.canonical {
		j.computeAlwaysDistances(grid)
		return nil
	}

	// Phase 1: detect primary jump points (forced neighbors)
	primaryJPs := j.detectPrimaryJumpPoints(grid)

	// Phase 2: compute cardinal and diagonal distances
	j.computeDistances(grid, primaryJPs)
	return nil
}

// AdaptiveDensityThreshold is the obstacle ratio at or above which Adaptive
// JPS+ prefers A*.
const AdaptiveDensityThreshold = 0.1

func (j *JPSPlusFinder) useAStar(grid Grid) bool {
	oc, ok := grid.(interface{ ObstacleCount() int32 })
	if !ok {
		return false
	}
	area := grid.Width() * grid.Height()
	if area == 0 {
		return false
	}
	return float32(oc.ObstacleCount())/float32(area) >= AdaptiveDensityThreshold
}

func (j *JPSPlusFinder) newAdaptiveFallback() *AStarFinder {
	return NewAStarFinder(
		WithDiagonal(j.DiagonalMovement),
		WithHeuristic(j.Heuristic),
		WithWeight(j.Weight),
	)
}

func (j *JPSPlusFinder) preflight(startX, startY, endX, endY int32, grid Grid) (path [][2]int32, err error) {
	if !grid.IsInside(startX, startY) || !grid.IsInside(endX, endY) {
		return nil, ErrOutOfBounds
	}
	if !supportsJPS(grid) {
		return nil, ErrUnsupportedGrid
	}
	if path, err = checkEndpoints(startX, startY, endX, endY, grid); path != nil || err != nil {
		return path, err
	}
	if !j.hasTable() {
		panic("JPSPlusFinder: Precompute(grid) must be called before FindPath")
	}
	if grid.Width() != j.width || grid.Height() != j.height ||
		(j.canonical && j.DiagonalMovement != DiagonalAlways) {
		return nil, ErrPrecomputedMismatch
	}
	return nil, nil
}

// FindPath performs JPS+ pathfinding using the precomputed jump table.
// Precompute(grid) must have been called first with the same grid.
func (j *JPSPlusFinder) FindPath(startX, startY, endX, endY int32, grid Grid, dst [][2]int32) ([][2]int32, error) {
	if path, err := j.preflight(startX, startY, endX, endY, grid); path != nil || err != nil {
		return finishPath(path, dst, true), err
	}
	fingerprintGrid, canValidate := grid.(WalkabilityFingerprintGrid)
	useTable := j.hasFingerprint && canValidate
	if useTable && fingerprintGrid.WalkabilityFingerprint() != j.fingerprint {
		return nil, ErrPrecomputedMismatch
	}
	if j.Adaptive && j.useAStar(grid) {
		return j.newAdaptiveFallback().FindPath(startX, startY, endX, endY, grid, dst)
	}
	// Legacy jumps for the at-most-one-obstacle rule can miss optimal turns.
	if !useTable || j.DiagonalMovement == DiagonalIfAtMostOneObstacle {
		fallback := NewJumpPointFinder(WithDiagonal(j.DiagonalMovement), WithHeuristic(j.Heuristic), WithWeight(j.Weight))
		return fallback.FindPath(startX, startY, endX, endY, grid, dst)
	}
	searchSeq := globalSearchSeq.Add(1)
	states, err := acquireStates(grid.Width(), grid.Height(), searchSeq)
	if err != nil {
		return nil, err
	}
	defer releaseStates(states)
	if j.canonical {
		p, err := findJPSPath(startX, startY, endX, endY, grid, states, jpsSearchOptions{
			heuristic: j.Heuristic, weight: j.Weight,
			neighbors: alwaysFindNeighbors, jump: alwaysTableJump, table: &j.jumpTable,
		})
		return finishPath(p, dst, false), err
	}
	p, err := j.findPath(startX, startY, endX, endY, grid, states)
	return finishPath(p, dst, false), err
}

func (j *JPSPlusFinder) findPath(startX, startY, endX, endY int32, grid Grid, states *searchStates) ([][2]int32, error) {
	openList := &minHeap{nodes: states.heap[:0], state: states}
	defer func() {
		clear(openList.nodes[:cap(openList.nodes)])
		states.heap = openList.nodes[:0]
	}()
	pathBuf := states.pathBuf[:0]
	expandBuf := states.expandBuf[:0]
	defer func() {
		states.pathBuf = pathBuf
		states.expandBuf = expandBuf
	}()

	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)
	if startNode == nil || endNode == nil {
		return nil, nil
	}

	startState := states.get(startNode)
	states.get(endNode)
	walk := newWalkableView(grid)
	startState.G = 0
	startState.F = 0
	startIdx := states.index(startNode)
	endIdx := states.index(endNode)
	openList.Push(startIdx, startState.F)

	for !openList.Empty() {
		nodeIdx := openList.Pop()
		node := grid.GetNodeAt(nodeIdx%states.width, nodeIdx/states.width)
		states.states[nodeIdx].Closed = true

		if nodeIdx == endIdx {
			pathBuf = states.backtrace(node, pathBuf)
			expandBuf = expandPathInto(expandBuf, pathBuf)
			return expandBuf, nil
		}

		j.identifySuccessors(node, endNode, nodeIdx, endIdx, states, openList, &walk)
	}
	return nil, nil
}

func (j *JPSPlusFinder) identifySuccessors(node, endNode *Node, nodeIdx, endIdx int32, states *searchStates, openList *minHeap, walk *walkableView) {
	x, y := node.X, node.Y
	endX, endY := endNode.X, endNode.Y
	heuristic := j.Heuristic
	// The current node was just popped and the end state was initialized before the search.
	nodeG := states.states[nodeIdx].G
	endState := &states.states[endIdx]

	// Use diagonal movement setting to determine which directions to check
	dirs := j.dirs

	// relaxAdjacent offers a one-step move in (dx,dy). It keeps the search
	// complete and optimal when the static jump table cannot encode every
	// goal-dependent jump point.
	relaxAdjacent := func(dx, dy int32) {
		if dx != 0 && dy != 0 && !j.diagonalStepAllowed(walk, x, y, dx, dy) {
			return
		}
		nx, ny := x+dx, y+dy
		if !walk.at(nx, ny) {
			return
		}
		adjIdx := ny*states.width + nx
		ns := states.at(adjIdx)
		if ns.Closed {
			return
		}
		var moveCost float32 = 1
		if dx != 0 && dy != 0 {
			moveCost = SQRT2
		}
		ng := nodeG + moveCost
		if ns.Opened == 0 || ng < ns.G {
			ns.G = ng
			if ns.Opened == 0 {
				ns.H = heuristic(float32(AbsInt(nx-endX)), float32(AbsInt(ny-endY)))
			}
			ns.F = ns.G + ns.H*j.Weight
			ns.Parent = nodeIdx
			if ns.Opened == 0 {
				openList.Push(adjIdx, ns.F)
				ns.Opened = 1
			} else {
				openList.UpdateItem(adjIdx, ns.F)
			}
		}
	}

	for _, dirIdx := range dirs {
		dist := j.getDist(x, y, dirIdx)
		dx, dy := dirToVec(dirIdx)

		// Check if the end node lies in this direction
		if endInDirection(x, y, endX, endY, dx, dy) {
			endDist := absDirDist(endX-x, endY-y, dx, dy)
			obstacleDist := int32(-dist) // positive when no jump point (dist < 0)
			if (dist <= 0 && endDist <= obstacleDist) || (dist > 0 && endDist <= int32(dist)) {
				// End is reachable in this direction, add it as successor
				gd := octileDist(AbsInt(endX-x), AbsInt(endY-y))
				ens := endState
				ng := nodeG + gd
				if !ens.Closed && (ens.Opened == 0 || ng < ens.G) {
					ens.G = ng
					if ens.Opened == 0 {
						ens.H = 0
					}
					ens.F = ng
					ens.Parent = nodeIdx
					if ens.Opened == 0 {
						openList.Push(endIdx, ens.F)
						ens.Opened = 1
					}
				}
				continue
			}
		}

		if dist <= 0 {
			// No jump point in this direction — take a normal step.
			relaxAdjacent(dx, dy)
			continue
		}

		// Positive dist: jump point exists at this distance
		jx, jy := x+dx*int32(dist), y+dy*int32(dist)

		if !walk.at(jx, jy) {
			continue
		}

		jumpIdx := jy*states.width + jx
		ns := states.at(jumpIdx)
		if ns.Closed {
			continue
		}

		gd := octileDist(AbsInt(jx-x), AbsInt(jy-y))
		ng := nodeG + gd

		if ns.Opened == 0 || ng < ns.G {
			ns.G = ng
			if ns.Opened == 0 {
				ns.H = heuristic(float32(AbsInt(jx-endX)), float32(AbsInt(jy-endY)))
			}
			ns.F = ns.G + ns.H*j.Weight
			ns.Parent = nodeIdx

			if ns.Opened == 0 {
				openList.Push(jumpIdx, ns.F)
				ns.Opened = 1
			} else {
				openList.UpdateItem(jumpIdx, ns.F)
			}
		}

		// Also allow turning at the intermediate cell; the static table cannot
		// represent jump points that only exist because of the current goal.
		if dist > 1 {
			relaxAdjacent(dx, dy)
		}
	}
}

// diagonalStepAllowed applies the configured diagonal movement rule to a
// one-step diagonal move from (x, y).
func (j *JPSPlusFinder) diagonalStepAllowed(walk *walkableView, x, y, dx, dy int32) bool {
	switch j.DiagonalMovement {
	case DiagonalNever:
		return false
	case DiagonalOnlyWhenNoObstacles:
		return walk.at(x+dx, y) && walk.at(x, y+dy)
	case DiagonalIfAtMostOneObstacle:
		return walk.at(x+dx, y) || walk.at(x, y+dy)
	default: // DiagonalAlways
		return true
	}
}

// endInDirection checks if (ex,ey) is reachable in direction (dx,dy) from (x,y).
func endInDirection(x, y, ex, ey, dx, dy int32) bool {
	rx, ry := ex-x, ey-y
	if dx == 0 && dy == 0 {
		return false
	}
	if dx == 0 {
		return rx == 0 && ry*dy > 0 && ry%dy == 0
	}
	if dy == 0 {
		return ry == 0 && rx*dx > 0 && rx%dx == 0
	}
	return rx*dx > 0 && ry*dy > 0 && rx/dx == ry/dy && rx%dx == 0 && ry%dy == 0
}

// absDirDist returns the number of steps from (rx,ry) in direction (dx,dy).
func absDirDist(rx, ry, dx, dy int32) int32 {
	if dx != 0 {
		return AbsInt(rx / dx)
	}
	return AbsInt(ry / dy)
}

func octileDist(dx, dy int32) float32 {
	f := SQRT2 - 1
	if dx < dy {
		return f*float32(dx) + float32(dy)
	}
	return f*float32(dy) + float32(dx)
}

func dirToVec(dirIdx int) (int32, int32) {
	switch dirIdx {
	case dirN:
		return 0, -1
	case dirNE:
		return 1, -1
	case dirE:
		return 1, 0
	case dirSE:
		return 1, 1
	case dirS:
		return 0, 1
	case dirSW:
		return -1, 1
	case dirW:
		return -1, 0
	case dirNW:
		return -1, -1
	default:
		return 0, 0
	}
}
