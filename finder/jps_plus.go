package finder

import (
	"encoding/binary"
	"errors"
	"fmt"
)

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

// Jump-table format magics: JPS (int8) and JPS2 (int16).
const (
	jpsPlusMagic8  uint32 = 0x4A5053
	jpsPlusMagic16 uint32 = 0x4A505332
)

var jpsCardinalDirs = [4][2]int{
	{0, -1}, // N
	{1, 0},  // E
	{0, 1},  // S
	{-1, 0}, // W
}

var jpsDiagonalDirs = [4][2]int{
	{1, -1},  // NE
	{1, 1},   // SE
	{-1, 1},  // SW
	{-1, -1}, // NW
}

// jpsCardinalIdx maps cardinal direction vectors to dir index.
var jpsCardinalIdx = map[[2]int]int{
	{0, -1}: dirN,
	{1, 0}:  dirE,
	{0, 1}:  dirS,
	{-1, 0}: dirW,
}

// jpsDiagonalIdx maps diagonal direction vectors to dir index.
var jpsDiagonalIdx = map[[2]int]int{
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
// The static table cannot encode jump points that depend on the goal, so the
// search also relaxes one-step moves from every expanded node. This keeps the
// result identical to A* for all diagonal modes while retaining the jump-table
// acceleration on open maps.
//
// JPS+ assumes every step has the same cost (tile Weight == 1). Tile weights are
// ignored; use A* or Dijkstra for weighted grids.
//
// JPS+ only supports axis-aligned grids (OrthogonalGrid). Hex and staggered
// grids return ErrUnsupportedGrid.
type JPSPlusFinder struct {
	Heuristic        HeuristicFunc
	DiagonalMovement DiagonalMovement
	Weight           float64
	Adaptive         bool

	width, height int

	// jump8/jump16 store precomputed jump distances per cell per direction.
	// Layout: (y*width + x)*8 + dir
	// Positive = jump point at that distance; negative = obstacle at abs(distance).
	// int8 is used when both dimensions are <= 128; int16 otherwise.
	jump8  []int8
	jump16 []int16
	wide   bool

	// Runtime state
	grid             Grid
	startNode        *Node
	endNode          *Node
	openList         *MinHeap
	heapSlice        []*Node
	pathBuf          [][2]int
	expandBuf        [][2]int
	adaptiveFallback *AStarFinder

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
	j.precompute(grid, grid.Width() > 128 || grid.Height() > 128)
	return nil
}

// precompute builds the table with an explicit cell width. It is split out so
// tests can compare the int8 and int16 encodings of the same grid.
func (j *JPSPlusFinder) precompute(grid Grid, wide bool) {
	j.width = grid.Width()
	j.height = grid.Height()
	j.wide = wide
	size := j.width * j.height * 8
	if j.wide {
		if cap(j.jump16) < size {
			j.jump16 = make([]int16, size)
		} else {
			j.jump16 = j.jump16[:size]
			clear(j.jump16)
		}
		j.jump8 = nil
	} else {
		if cap(j.jump8) < size {
			j.jump8 = make([]int8, size)
		} else {
			j.jump8 = j.jump8[:size]
			clear(j.jump8)
		}
		j.jump16 = nil
	}

	// Phase 1: detect primary jump points (forced neighbors)
	primaryJPs := j.detectPrimaryJumpPoints(grid)

	// Phase 2: compute cardinal and diagonal distances
	j.computeDistances(grid, primaryJPs)
}

// detectPrimaryJumpPoints scans each walkable cell for forced neighbors.
// Returns a per-cell bitmask of directions where the cell acts as a jump point.
func (j *JPSPlusFinder) detectPrimaryJumpPoints(grid Grid) []uint8 {
	w, h := j.width, j.height
	primaryJPs := make([]uint8, w*h)

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}

			for _, dir := range jpsCardinalDirs {
				nx, ny := x+dir[0], y+dir[1]
				if !grid.IsInside(nx, ny) || grid.IsWalkableAt(nx, ny) {
					continue
				}

				// Perpendicular to wall direction
				var checkDirs [2][2]int
				if dir[0] != 0 { // horizontal direction (E/W)
					checkDirs = [2][2]int{{0, -1}, {0, 1}} // N/S
				} else { // vertical direction (N/S)
					checkDirs = [2][2]int{{-1, 0}, {1, 0}} // W/E
				}

				for _, cd := range checkDirs {
					// From the wall cell, check perpendicular
					wnx, wny := nx+cd[0], ny+cd[1]
					if !grid.IsInside(wnx, wny) || !grid.IsWalkableAt(wnx, wny) {
						continue
					}

					// Potential jump point from origin in perpendicular direction
					jpx, jpy := x+cd[0], y+cd[1]
					if !grid.IsInside(jpx, jpy) || !grid.IsWalkableAt(jpx, jpy) {
						continue
					}

					dirIdx := jpsCardinalIdx[cd]
					primaryJPs[jpy*w+jpx] |= 1 << dirIdx
				}
			}
		}
	}
	return primaryJPs
}

// computeDistances fills the jumpTable with distances in all 8 directions.
func (j *JPSPlusFinder) computeDistances(grid Grid, primaryJPs []uint8) {
	w, h := j.width, j.height

	// Helper: get distance to obstacle or jump point in a cardinal direction
	scanCardinal := func(x, y int, dx, dy int, dirIdx int) int16 {
		dist := 0
		for {
			nx, ny := x+dx*(dist+1), y+dy*(dist+1)
			if !grid.IsInside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
				return int16(-dist)
			}
			if primaryJPs[ny*w+nx]&(1<<dirIdx) != 0 {
				return int16(dist + 1)
			}
			dist++
		}
	}

	// Part A: Cardinal directions
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}
			base := (y*w + x) * 8
			for _, dir := range jpsCardinalDirs {
				dirIdx := jpsCardinalIdx[dir]
				j.setTable(base+dirIdx, scanCardinal(x, y, dir[0], dir[1], dirIdx))
			}
		}
	}

	// Part B: Diagonal directions
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}
			base := (y*w + x) * 8

			for _, ddir := range jpsDiagonalDirs {
				ddirIdx := jpsDiagonalIdx[ddir]
				hdx, hdy := ddir[0], 0 // horizontal component
				vdx, vdy := 0, ddir[1] // vertical component
				hDirIdx := jpsCardinalIdx[[2]int{hdx, hdy}]
				vDirIdx := jpsCardinalIdx[[2]int{vdx, vdy}]

				dist := 0
				for {
					cx, cy := x+ddir[0]*dist, y+ddir[1]*dist
					if !grid.IsInside(cx, cy) || !grid.IsWalkableAt(cx, cy) {
						j.setTable(base+ddirIdx, int16(-dist))
						break
					}

					if dist > 0 {
						// Check cardinal neighbors at this diagonal position
						cb := (cy*w + cx) * 8
						if j.getTable(cb+hDirIdx) > 0 || j.getTable(cb+vDirIdx) > 0 {
							j.setTable(base+ddirIdx, int16(dist))
							break
						}
					}

					// Check horizontal and vertical neighbors are walkable
					hnx, hny := cx+hdx, cy+hdy
					vnx, vny := cx+vdx, cy+vdy
					if !grid.IsInside(hnx, hny) || !grid.IsWalkableAt(hnx, hny) {
						j.setTable(base+ddirIdx, int16(-dist))
						break
					}
					if !grid.IsInside(vnx, vny) || !grid.IsWalkableAt(vnx, vny) {
						j.setTable(base+ddirIdx, int16(-dist))
						break
					}

					// Next diagonal step
					nx, ny := cx+ddir[0], cy+ddir[1]
					if !grid.IsInside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
						j.setTable(base+ddirIdx, int16(-(dist + 1)))
						break
					}
					dist++
				}
			}
		}
	}
}

// hasTable reports whether Precompute (or LoadPrecomputed) has run.
func (j *JPSPlusFinder) hasTable() bool { return j.jump8 != nil || j.jump16 != nil }

func (j *JPSPlusFinder) getTable(i int) int16 {
	if j.wide {
		return j.jump16[i]
	}
	return int16(j.jump8[i])
}

func (j *JPSPlusFinder) setTable(i int, v int16) {
	if j.wide {
		j.jump16[i] = v
	} else {
		j.jump8[i] = int8(v)
	}
}

// getDist returns the precomputed jump distance for cell (x,y) in direction dirIdx.
func (j *JPSPlusFinder) getDist(x, y, dirIdx int) int16 {
	if !j.grid.IsInside(x, y) {
		return 0
	}
	i := (y*j.width+x)*8 + dirIdx
	if j.wide {
		return j.jump16[i]
	}
	return int16(j.jump8[i])
}

// PrecomputedData serializes the jump table to binary data.
// The returned slice can be saved to a file and loaded later with LoadPrecomputed,
// avoiding the cost of re-running Precompute on server restart.
func (j *JPSPlusFinder) PrecomputedData() ([]byte, error) {
	if !j.hasTable() {
		return nil, errors.New("JPSPlusFinder: Precompute must be called before Export")
	}
	magic := jpsPlusMagic8
	cellBytes := 1
	if j.wide {
		magic = jpsPlusMagic16
		cellBytes = 2
	}
	// Format: magic(4) + width(4) + height(4) + table
	size := 12 + j.width*j.height*8*cellBytes
	buf := make([]byte, size)
	binary.LittleEndian.PutUint32(buf[0:4], magic)
	binary.LittleEndian.PutUint32(buf[4:8], uint32(j.width))
	binary.LittleEndian.PutUint32(buf[8:12], uint32(j.height))
	if j.wide {
		for i, v := range j.jump16 {
			binary.LittleEndian.PutUint16(buf[12+i*2:], uint16(v))
		}
	} else {
		for i, v := range j.jump8 {
			buf[12+i] = byte(v)
		}
	}
	return buf, nil
}

// LoadPrecomputed loads previously exported jump table data.
// Must be called before FindPath with a grid of matching dimensions.
func (j *JPSPlusFinder) LoadPrecomputed(data []byte) error {
	if len(data) < 12 {
		return errors.New("JPSPlusFinder: precomputed data too short")
	}
	magic := binary.LittleEndian.Uint32(data[0:4])
	var wide bool
	switch magic {
	case jpsPlusMagic8:
	case jpsPlusMagic16:
		wide = true
	default:
		return fmt.Errorf("JPSPlusFinder: invalid magic 0x%X", magic)
	}
	w := int(binary.LittleEndian.Uint32(data[4:8]))
	h := int(binary.LittleEndian.Uint32(data[8:12]))
	tableSize := w * h * 8
	cellBytes := 1
	if wide {
		cellBytes = 2
	}
	if len(data)-12 < tableSize*cellBytes {
		return fmt.Errorf("JPSPlusFinder: precomputed data too short, need %d bytes", tableSize*cellBytes)
	}
	j.width = w
	j.height = h
	j.wide = wide
	if wide {
		if cap(j.jump16) < tableSize {
			j.jump16 = make([]int16, tableSize)
		} else {
			j.jump16 = j.jump16[:tableSize]
		}
		j.jump8 = nil
		for i := 0; i < tableSize; i++ {
			j.jump16[i] = int16(binary.LittleEndian.Uint16(data[12+i*2:]))
		}
	} else {
		if cap(j.jump8) < tableSize {
			j.jump8 = make([]int8, tableSize)
		} else {
			j.jump8 = j.jump8[:tableSize]
		}
		j.jump16 = nil
		for i := 0; i < tableSize; i++ {
			j.jump8[i] = int8(data[12+i])
		}
	}
	return nil
}

// PrecomputedSize returns the byte size of the serialized jump table.
// Useful for estimating storage or memory before exporting.
func (j *JPSPlusFinder) PrecomputedSize() int {
	if !j.hasTable() {
		return 0
	}
	cellBytes := 1
	if j.wide {
		cellBytes = 2
	}
	return 12 + j.width*j.height*8*cellBytes
}

// AdaptiveDensityThreshold is the obstacle ratio at or above which Adaptive
// JPS+ prefers A*.
const AdaptiveDensityThreshold = 0.1

func (j *JPSPlusFinder) useAStar(grid Grid) bool {
	oc, ok := grid.(interface{ ObstacleCount() int })
	if !ok {
		return false
	}
	area := grid.Width() * grid.Height()
	if area == 0 {
		return false
	}
	return float64(oc.ObstacleCount())/float64(area) >= AdaptiveDensityThreshold
}

func (j *JPSPlusFinder) newAdaptiveFallback() Finder {
	if j.adaptiveFallback == nil {
		j.adaptiveFallback = NewAStarFinder(
			WithDiagonal(j.DiagonalMovement),
			WithHeuristic(j.Heuristic),
			WithWeight(j.Weight),
		)
	}
	return j.adaptiveFallback
}

// FindPath performs JPS+ pathfinding using the precomputed jump table.
// Precompute(grid) must have been called first with the same grid.
func (j *JPSPlusFinder) FindPath(startX, startY, endX, endY int, grid Grid) ([][2]int, error) {
	// GetNodeAt panics on an out-of-range index, so validate up front and
	// report it as a caller error instead of a crash.
	if !grid.IsInside(startX, startY) || !grid.IsInside(endX, endY) {
		return nil, ErrOutOfBounds
	}
	if !supportsJPS(grid) {
		return nil, ErrUnsupportedGrid
	}
	if path, err := checkEndpoints(startX, startY, endX, endY, grid); path != nil || err != nil {
		return path, err
	}
	if !j.hasTable() {
		panic("JPSPlusFinder: Precompute(grid) must be called before FindPath")
	}
	if grid.Width() != j.width || grid.Height() != j.height {
		return nil, ErrPrecomputedMismatch
	}
	if j.Adaptive && j.useAStar(grid) {
		return j.newAdaptiveFallback().FindPath(startX, startY, endX, endY, grid)
	}
	searchSeq := int(globalSearchSeq.Add(1))
	j.grid = grid
	j.openList = &MinHeap{nodes: j.heapSlice[:0]}
	defer func() { j.heapSlice = j.openList.nodes[:0] }()

	j.startNode = grid.GetNodeAt(startX, startY)
	j.endNode = grid.GetNodeAt(endX, endY)
	if j.startNode == nil || j.endNode == nil {
		return nil, nil
	}

	j.startNode.ResetSearch(searchSeq)
	j.endNode.ResetSearch(searchSeq)
	j.startNode.G = 0
	j.startNode.F = 0
	j.openList.Push(j.startNode)

	for !j.openList.Empty() {
		node := j.openList.Pop()
		node.Closed = true

		if node.X == endX && node.Y == endY {
			j.pathBuf = j.pathBuf[:0]
			for n := node; n != nil; n = n.Parent {
				j.pathBuf = append(j.pathBuf, [2]int{n.X, n.Y})
			}
			for i, k := 0, len(j.pathBuf)-1; i < k; i, k = i+1, k-1 {
				j.pathBuf[i], j.pathBuf[k] = j.pathBuf[k], j.pathBuf[i]
			}
			j.expandBuf = j.expandBuf[:0]
			for i := 0; i < len(j.pathBuf)-1; i++ {
				x0, y0 := j.pathBuf[i][0], j.pathBuf[i][1]
				x1, y1 := j.pathBuf[i+1][0], j.pathBuf[i+1][1]
				dx := x1 - x0
				dy := y1 - y0
				var sx, sy int
				if dx < 0 {
					dx = -dx
					sx = -1
				} else {
					sx = 1
				}
				if dy < 0 {
					dy = -dy
					sy = -1
				} else {
					sy = 1
				}
				err := dx - dy
				for {
					if x0 == x1 && y0 == y1 {
						break
					}
					j.expandBuf = append(j.expandBuf, [2]int{x0, y0})
					e2 := 2 * err
					if e2 > -dy {
						err -= dy
						x0 += sx
					}
					if e2 < dx {
						err += dx
						y0 += sy
					}
				}
			}
			j.expandBuf = append(j.expandBuf, j.pathBuf[len(j.pathBuf)-1])
			return j.expandBuf, nil
		}

		j.identifySuccessors(node, searchSeq)
	}
	return nil, nil
}

func (j *JPSPlusFinder) identifySuccessors(node *Node, searchSeq int) {
	x, y := node.X, node.Y
	endX, endY := j.endNode.X, j.endNode.Y
	heuristic := j.Heuristic
	grid := j.grid
	openList := j.openList

	// Use diagonal movement setting to determine which directions to check
	dirs := j.dirs

	// relaxAdjacent offers a one-step move in (dx,dy). It keeps the search
	// complete and optimal when the static jump table cannot encode every
	// goal-dependent jump point.
	relaxAdjacent := func(dx, dy int) {
		if dx != 0 && dy != 0 && !j.diagonalStepAllowed(x, y, dx, dy) {
			return
		}
		nx, ny := x+dx, y+dy
		if !grid.IsInside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
			return
		}
		adjNode := grid.GetNodeAt(nx, ny)
		adjNode.ResetSearch(searchSeq)
		if adjNode.Closed {
			return
		}
		var moveCost float64 = 1
		if dx != 0 && dy != 0 {
			moveCost = SQRT2
		}
		ng := node.G + moveCost
		if adjNode.Opened == 0 || ng < adjNode.G {
			adjNode.G = ng
			if adjNode.Opened == 0 {
				adjNode.H = heuristic(float64(AbsInt(nx-endX)), float64(AbsInt(ny-endY)))
			}
			adjNode.F = adjNode.G + adjNode.H*j.Weight
			adjNode.Parent = node
			if adjNode.Opened == 0 {
				openList.Push(adjNode)
				adjNode.Opened = 1
			} else {
				openList.UpdateItem(adjNode)
			}
		}
	}

	for _, dirIdx := range dirs {
		dist := j.getDist(x, y, dirIdx)
		dx, dy := dirToVec(dirIdx)

		// Check if the end node lies in this direction
		if endInDirection(x, y, endX, endY, dx, dy) {
			endDist := absDirDist(endX-x, endY-y, dx, dy)
			obstacleDist := int(-dist) // positive when no jump point (dist < 0)
			if (dist <= 0 && endDist <= obstacleDist) || (dist > 0 && endDist <= int(dist)) {
				// End is reachable in this direction, add it as successor
				gd := octileDist(AbsInt(endX-x), AbsInt(endY-y))
				endNode := j.endNode
				ng := node.G + gd
				endNode.ResetSearch(searchSeq)
				if !endNode.Closed && (endNode.Opened == 0 || ng < endNode.G) {
					endNode.G = ng
					if endNode.Opened == 0 {
						endNode.H = 0
					}
					endNode.F = ng
					endNode.Parent = node
					if endNode.Opened == 0 {
						openList.Push(endNode)
						endNode.Opened = 1
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
		jx, jy := x+dx*int(dist), y+dy*int(dist)

		if !grid.IsInside(jx, jy) || !grid.IsWalkableAt(jx, jy) {
			continue
		}

		jumpNode := grid.GetNodeAt(jx, jy)
		jumpNode.ResetSearch(searchSeq)

		if jumpNode.Closed {
			continue
		}

		gd := octileDist(AbsInt(jx-x), AbsInt(jy-y))
		ng := node.G + gd

		if jumpNode.Opened == 0 || ng < jumpNode.G {
			jumpNode.G = ng
			if jumpNode.Opened == 0 {
				jumpNode.H = heuristic(float64(AbsInt(jx-endX)), float64(AbsInt(jy-endY)))
			}
			jumpNode.F = jumpNode.G + jumpNode.H*j.Weight
			jumpNode.Parent = node

			if jumpNode.Opened == 0 {
				openList.Push(jumpNode)
				jumpNode.Opened = 1
			} else {
				openList.UpdateItem(jumpNode)
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
func (j *JPSPlusFinder) diagonalStepAllowed(x, y, dx, dy int) bool {
	grid := j.grid
	switch j.DiagonalMovement {
	case DiagonalNever:
		return false
	case DiagonalOnlyWhenNoObstacles:
		return grid.IsWalkableAt(x+dx, y) && grid.IsWalkableAt(x, y+dy)
	case DiagonalIfAtMostOneObstacle:
		return grid.IsWalkableAt(x+dx, y) || grid.IsWalkableAt(x, y+dy)
	default: // DiagonalAlways
		return true
	}
}

// endInDirection checks if (ex,ey) is reachable in direction (dx,dy) from (x,y).
func endInDirection(x, y, ex, ey, dx, dy int) bool {
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
func absDirDist(rx, ry, dx, dy int) int {
	if dx != 0 {
		return AbsInt(rx / dx)
	}
	return AbsInt(ry / dy)
}

func octileDist(dx, dy int) float64 {
	f := SQRT2 - 1
	if dx < dy {
		return f*float64(dx) + float64(dy)
	}
	return f*float64(dy) + float64(dx)
}

func dirToVec(dirIdx int) (int, int) {
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
